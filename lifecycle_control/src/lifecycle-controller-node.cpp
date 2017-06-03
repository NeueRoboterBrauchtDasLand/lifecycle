#include <map>
#include <vector>
#include <string>

#include <ros/ros.h>

#include <lifecycle_msgs/NodeStatusArray.h>
#include <lifecycle_msgs/Lifecycle.h>
#include <lifecycle_msgs/LifecycleControllerAction.h>

#include "NodeHistory.h"

std::map<std::string, std::size_t> _nodeIdx;
std::map<std::string, std::vector<std::size_t>> _groupIdxs;
std::vector<std::shared_ptr<lifecycle_controll::NodeHistory>> _histories;
std::vector<ros::Time> _lastStatus;
std::vector<std::shared_ptr<ros::ServiceClient>> _srvsNodeAction;

std::unique_ptr<ros::NodeHandle> _nh;
std::unique_ptr<ros::Publisher> _pubNodeStates;

void nodeStatusCallback(const lifecycle_msgs::NodeStatus& msg)
{
    auto index = _nodeIdx.find(msg.node_name);

    // Node name is new.
    if (index == _nodeIdx.end())
    {
        _nodeIdx.insert(std::pair<std::string, std::size_t>(msg.node_name, _histories.size()));
        _histories.push_back(std::make_shared<lifecycle_controll::NodeHistory>());
        _lastStatus.push_back(msg.stamp);


        _srvsNodeAction.push_back(std::make_shared<ros::ServiceClient>());
        *(_srvsNodeAction.back()) = _nh->serviceClient<lifecycle_msgs::Lifecycle>("/lifecycle/service/" + msg.node_name);
        _histories.back()->insert(lifecycle_msgs::cpp::NodeStatus(msg));

        // Check if the group is also new.
        auto groupIt = _groupIdxs.find(msg.group);

        // Group is new. Add it to the group container.
        if (groupIt == _groupIdxs.end())
        {
            std::vector<std::size_t> indices;
            indices.push_back(_nodeIdx[msg.node_name]);
            _groupIdxs.insert(std::pair<std::string, std::vector<std::size_t>>(msg.group, indices));
        }
        // Group is known. Add node index to the indices container of the group.
        else
        {
            groupIt->second.push_back(_nodeIdx[msg.node_name]);
        }
    }
    // Node name is known. Insert state to history and update lastStatus.
    else
    {
        _histories[index->second]->insert(lifecycle_msgs::cpp::NodeStatus(msg));
        _lastStatus[index->second] = msg.stamp;
    }
}

bool callbackService(lifecycle_msgs::LifecycleControllerAction::Request& req,
                     lifecycle_msgs::LifecycleControllerAction::Response& res)
{
    switch (req.action)
    {
    case lifecycle_msgs::LifecycleControllerAction::Request::ACTION_CHANGE_LIFECYCLE:
        {
            auto index = _nodeIdx.find(req.target_node);
            auto groupIt = _groupIdxs.find(req.target_node);

            if (req.target_node != "broadcast" && index == _nodeIdx.end() && groupIt == _groupIdxs.end())
            {
                ROS_ERROR_STREAM(ros::this_node::getName() + ": target node " + req.target_node + " unkown.");
                return false;
            }

            lifecycle_msgs::Lifecycle msg;

            msg.request.action = lifecycle_msgs::Lifecycle::Request::ACTION_GO_IN_STATE;
            msg.request.lifecycle = req.target_lifecycle;

            if (req.target_node == "broadcast")
            {
                bool ret = true;

                for (auto& client : _srvsNodeAction)
                    ret &= client->call(msg);

                return ret;
            }
            else if (groupIt != _groupIdxs.end())
            {
                bool ret = true;

                for (auto& nodeIdx : groupIt->second)
                    ret &= _srvsNodeAction[nodeIdx]->call(msg);

                return ret;
            }
            else if (!_srvsNodeAction[index->second]->call(msg))
            {
                ROS_ERROR_STREAM(ros::this_node::getName() + ": node '" + req.target_node + "' rejects the serivce request.");
                return false;
            }

            return true;
        }

    default:
        ROS_WARN_STREAM(ros::this_node::getName() + ": requested action is unkown.");
        return false;
    }

    return true;
}

void callbackTimer(const ros::TimerEvent&)
{
    lifecycle_msgs::NodeStatusArray msg;

    msg.states.resize(_nodeIdx.size());

    for (auto& history : _histories)
        history->update();

    for (unsigned int i = 0; i < _nodeIdx.size(); ++i)
        msg.states[i] = _histories[i]->lastStatus().toMsg();

    for (const auto& groupIt : _groupIdxs)
        msg.groups.push_back(groupIt.first);

    _pubNodeStates->publish(msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lifecycle_controller");
    _nh = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle);
    _pubNodeStates = std::unique_ptr<ros::Publisher>(new ros::Publisher);
    *_pubNodeStates = _nh->advertise<lifecycle_msgs::NodeStatusArray>("/lifecycle/controller/node_states", 1);
    ros::Subscriber subStatus(_nh->subscribe("/lifecycle/status", 100, nodeStatusCallback));
    ros::ServiceServer srv(_nh->advertiseService("/lifecycle/service/controller", callbackService));
    ros::Timer timer = _nh->createTimer(ros::Duration(2.0), callbackTimer);

    ros::spin();
}
