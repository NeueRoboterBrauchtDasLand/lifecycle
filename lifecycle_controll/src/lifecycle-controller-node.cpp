#include <map>
#include <vector>
#include <string>

#include <ros/ros.h>

#include <lifecycle_msgs/NodeStatusArray.h>
#include <lifecycle_msgs/Lifecycle.h>
#include <lifecycle_msgs/LifecycleControllerAction.h>

#include "NodeHistory.h"

std::map<std::string, std::size_t> _nodeIdx;
std::vector<std::shared_ptr<lifecycle_controll::NodeHistory>> _histories;
std::vector<ros::Time> _lastStatus;
std::vector<std::shared_ptr<ros::ServiceClient>> _srvsNodeAction;

std::unique_ptr<ros::NodeHandle> _nh;
std::unique_ptr<ros::Publisher> _pubNodeStates;

void nodeStatusCallback(const lifecycle_msgs::NodeStatus& msg)
{
    auto index = _nodeIdx.find(msg.node_name);

    if (index == _nodeIdx.end())
    {
        _nodeIdx.insert(std::pair<std::string, std::size_t>(msg.node_name, _histories.size()));
        _histories.push_back(std::make_shared<lifecycle_controll::NodeHistory>());
        _lastStatus.push_back(msg.stamp);


        _srvsNodeAction.push_back(std::make_shared<ros::ServiceClient>());
        *(_srvsNodeAction.back()) = _nh->serviceClient<lifecycle_msgs::Lifecycle>("/lifecycle/service/" + msg.node_name);
        _histories.back()->insert(lifecycle_msgs::cpp::NodeStatus(msg));
    }
    else
    {
        _histories[index->second]->insert(lifecycle_msgs::cpp::NodeStatus(msg));
        _lastStatus[index->second] = msg.stamp;
    }
}

bool callbackService(lifecycle_msgs::LifecycleControllerAction::Request& req,
                     lifecycle_msgs::LifecycleControllerAction::Response& res)
{

}

void callbackTimer(const ros::TimerEvent&)
{
    lifecycle_msgs::NodeStatusArray msg;

    msg.states.resize(_nodeIdx.size());

    for (unsigned int i = 0; i < _nodeIdx.size(); ++i)
        msg.states[i] = _histories[i]->lastStatus().toMsg();

    _pubNodeStates->publish(msg);
}

void printAllNodeNames(void)
{
    for (const auto& node : _nodeIdx)
        std::cout << node.first << std::endl;
}

void updateAllHistories(void)
{
    for (auto& node : _histories)
        node->update();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lifecycle_controller");
    _nh = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle);
    _pubNodeStates = std::unique_ptr<ros::Publisher>(new ros::Publisher);
    *_pubNodeStates = _nh->advertise<lifecycle_msgs::NodeStatusArray>("/lifecycle/controller/node_states", 1);
    ros::Subscriber subStatus(_nh->subscribe("/lifecycle/status", 100, nodeStatusCallback));
    ros::Timer timer = _nh->createTimer(ros::Duration(2.0), callbackTimer);

    ros::Rate rate(1.0);

    while (ros::ok())
    {
        updateAllHistories();
        printAllNodeNames();

        ros::spinOnce();
        rate.sleep();
    }
}
