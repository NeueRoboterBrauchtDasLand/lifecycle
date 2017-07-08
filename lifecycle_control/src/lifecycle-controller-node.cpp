#include <map>
#include <vector>
#include <string>

#include <ros/ros.h>

#include <lifecycle_msgs/Lifecycle.h>
#include <lifecycle_msgs/LifecycleControllerAction.h>
#include <lifecycle_msgs/NodeStatusArray.h>

#include "NodeStateHandle.h"
#include "NodeActionHandle.h"

std::shared_ptr<ros::NodeHandle> _nh;
std::unique_ptr<ros::Publisher> _pubNodeStates;

std::shared_ptr<lifecycle_control::NodeStateHandle> _nodeStateHandler;
std::shared_ptr<lifecycle_control::NodeStateDatabase> _nodeStateDatabase;
std::shared_ptr<lifecycle_control::NodeActionHandle> _nodeActionHandler;

bool callbackService(lifecycle_msgs::LifecycleControllerAction::Request& req,
                     lifecycle_msgs::LifecycleControllerAction::Response& res)
{
    switch (req.action)
    {
    case lifecycle_msgs::LifecycleControllerAction::Request::ACTION_CHANGE_LIFECYCLE:

        // broadcast addresses all nodes. Once a serice call is rejected this request will also be rejected.
        if (req.target_node == "broadcast")
        {
            for (const auto& node : _nodeStateDatabase->getNodes())
                _nodeActionHandler->createAction(node, static_cast<lifecycle_msgs::cpp::NodeStatus::State>(req.target_lifecycle));
        }
        // Check a group is the target. Once a serice call is rejected this request will also be rejected.
        else if (_nodeStateDatabase->existsGroup(req.target_node))
        {
            for (const auto& node : _nodeStateDatabase->getNodes(req.target_node))
                _nodeActionHandler->createAction(node, static_cast<lifecycle_msgs::cpp::NodeStatus::State>(req.target_lifecycle));
        }
        // Call the target node's lifecycle service with the requested action (lifecycle change).
        else if (_nodeStateDatabase->selectNode(req.target_node))
        {
            _nodeActionHandler->createAction(req.target_node, static_cast<lifecycle_msgs::cpp::NodeStatus::State>(req.target_lifecycle));
        }
        else
        {
            ROS_ERROR_STREAM(ros::this_node::getName() + ": target is unkown. --> reject service.");
            return false;
        }
    break;

    default:
        ROS_WARN_STREAM(ros::this_node::getName() + ": requested action is unkown.");
        return false;
    }

    return true;
}

void callbackTimer(const ros::TimerEvent&)
{
    lifecycle_msgs::NodeStatusArray msg;
    const auto lastStates(_nodeStateDatabase->getLastStateOfNodes());

    msg.groups = _nodeStateDatabase->getGroups();
    msg.states.resize(lastStates.size());

    for (unsigned int i = 0; i < lastStates.size(); ++i)
        msg.states[i] = lastStates[i].toMsg();

    _pubNodeStates->publish(msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lifecycle_controller");

    _nh = std::make_shared<ros::NodeHandle>();
    _pubNodeStates = std::unique_ptr<ros::Publisher>(new ros::Publisher);
    *_pubNodeStates = _nh->advertise<lifecycle_msgs::NodeStatusArray>("/lifecycle/controller/node_states", 1);

    _nodeStateDatabase = std::make_shared<lifecycle_control::NodeStateDatabase>(300);
    _nodeStateHandler = std::make_shared<lifecycle_control::NodeStateHandle>(_nodeStateDatabase, *_nh);
    _nodeActionHandler = std::make_shared<lifecycle_control::NodeActionHandle>(_nh);

    _nodeStateHandler->registerEventActor(std::static_pointer_cast<lifecycle_control::NodeStateEventActor>(_nodeActionHandler));

    ros::Subscriber subStatus(_nh->subscribe("/lifecycle/status",
                                             100,
                                             &lifecycle_control::NodeStateHandle::nodeStatusCallback,
                                             _nodeStateHandler.get()));
    ros::ServiceServer srv(_nh->advertiseService("/lifecycle/service/controller", callbackService));
    ros::Timer timer = _nh->createTimer(ros::Duration(2.0), callbackTimer);

    ros::spin();
}
