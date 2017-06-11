#include <map>
#include <vector>
#include <string>

#include <ros/ros.h>

#include <lifecycle_msgs/Lifecycle.h>
#include <lifecycle_msgs/LifecycleControllerAction.h>
#include <lifecycle_msgs/NodeStatusArray.h>

#include "NodeStateHandle.h"

std::unique_ptr<ros::NodeHandle> _nh;
std::unique_ptr<ros::Publisher> _pubNodeStates;

std::shared_ptr<lifecycle_control::NodeStateHandle> _nodeStateHandler;
std::shared_ptr<lifecycle_control::NodeStateDatabase> _nodeStateDatabase;

// TODO: Add timeout functionality for the node service calls. Add error flags to the node status msg for indicating a timeout.
bool callbackService(lifecycle_msgs::LifecycleControllerAction::Request& req,
                     lifecycle_msgs::LifecycleControllerAction::Response& res)
{
/*
    switch (req.action)
    {
    case lifecycle_msgs::LifecycleControllerAction::Request::ACTION_CHANGE_LIFECYCLE:
        {
            const auto index   = _nodeIdx.find(req.target_node);
            const auto groupIt = _groupIdxs.find(req.target_node);

            // Check if target_node is valid.
            if (req.target_node != "broadcast" && index == _nodeIdx.end() && groupIt == _groupIdxs.end())
            {
                ROS_ERROR_STREAM(ros::this_node::getName() + ": target node " + req.target_node + " unkown.");
                return false;
            }

            lifecycle_msgs::Lifecycle msg;

            msg.request.action = lifecycle_msgs::Lifecycle::Request::ACTION_GO_IN_STATE;
            msg.request.lifecycle = req.target_lifecycle;

            // broadcast addresses all nodes. Once a serice call is rejected this request will also be rejected.
            if (req.target_node == "broadcast")
            {
                bool ret = true;

                for (auto& client : _srvsNodeAction)
                    ret &= client->call(msg);

                return ret;
            }
            // Check a group is the target. Once a serice call is rejected this request will also be rejected.
            else if (groupIt != _groupIdxs.end())
            {
                bool ret = true;

                for (const auto& nodeIdx : groupIt->second)
                    ret &= _srvsNodeAction[nodeIdx]->call(msg);

                return ret;
            }
            // Call the target node's lifecycle service with the requested action (lifecycle change).
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
*/
    return true;
}

void callbackTimer(const ros::TimerEvent&)
{
    lifecycle_msgs::NodeStatusArray msg;

    _pubNodeStates->publish(msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lifecycle_controller");

    _nh = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle);
    _pubNodeStates = std::unique_ptr<ros::Publisher>(new ros::Publisher);
    *_pubNodeStates = _nh->advertise<lifecycle_msgs::NodeStatusArray>("/lifecycle/controller/node_states", 1);

    _nodeStateDatabase = std::make_shared<lifecycle_control::NodeStateDatabase>(300);
    _nodeStateHandler = std::make_shared<lifecycle_control::NodeStateHandle>(_nodeStateDatabase);

    ros::Subscriber subStatus(_nh->subscribe("/lifecycle/status",
                                             100,
                                             &lifecycle_control::NodeStateHandle::nodeStatusCallback,
                                             _nodeStateHandler.get()));
    ros::ServiceServer srv(_nh->advertiseService("/lifecycle/service/controller", callbackService));
    ros::Timer timer = _nh->createTimer(ros::Duration(2.0), callbackTimer);

    ros::spin();
}
