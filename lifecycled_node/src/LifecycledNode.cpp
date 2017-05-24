#include "lifecycled_node/LifecycledNode.h"

#include <iostream>

namespace lifecycled_node {

LifecycledNode::LifecycledNode(void)
{
    _nodeState = NodeStatus::State::CREATED;
}

LifecycledNode::LifecycledNode(ros::NodeHandle& privNh, ros::NodeHandle& nh)
{
    _nodeState = NodeStatus::State::CREATED;
    this->initializeLifecycle(privNh, nh);
}

LifecycledNode::~LifecycledNode(void)
{
    _timer.stop();
}

std::string LifecycledNode::nodeStateName(const NodeStatus::State state)
{
    switch (state)
    {
    case NodeStatus::State::CREATED:
        return "CREATED";
    case NodeStatus::State::UNCONFIGURED:
        return "UNCONFIGURED";
    case NodeStatus::State::INACTIVE:
        return "INACTIVE";
    case NodeStatus::State::ACTIVE:
        return "ACTIVE";
    case NodeStatus::State::FINALIZED:
        return "FINALIZED";
    default:
         return "UNKOWN";
    }
}

void LifecycledNode::initializeLifecycle(ros::NodeHandle& privNh, ros::NodeHandle& nh)
{
    if (_nodeState != NodeStatus::State::CREATED)
    {
        ROS_WARN_STREAM(ros::this_node::getName() + " is already initialized. --> Do nothing and return.");
        return;
    }

    privNh.param<double>("lifecycle_processing_freq", _processingFreq, _processingFreq);

    _pubState = nh.advertise<lifecycle_msgs::NodeStatus>("/lifecycle/status", 1);
    _srvResponser = nh.advertiseService("/lifecycle/service/" + ros::this_node::getName(),
                                        &LifecycledNode::processServiceRequest,
                                        this);
    _timer = nh.createTimer(ros::Duration(_processingFreq), &LifecycledNode::processLifecycle, this);
    _nodeState = NodeStatus::State::UNCONFIGURED;
}

void LifecycledNode::onCleanup(void)
{
    this->cleanup();
    _nodeState = NodeStatus::State::UNCONFIGURED;
}

void LifecycledNode::onConfigure(void)
{
    this->configure();
    _nodeState = NodeStatus::State::INACTIVE;
}

void LifecycledNode::onActivating(void)
{
    this->activating();
    _nodeState = NodeStatus::State::ACTIVE;
}

void LifecycledNode::onDeactivating(void)
{
    this->deactivating();
    _nodeState = NodeStatus::State::INACTIVE;
}

void LifecycledNode::onShutdown(void)
{
    this->shuttingDown();
    _nodeState = NodeStatus::State::FINALIZED;
}

void LifecycledNode::processLifecycle(const ros::TimerEvent& event)
{
    switch (_doExecute)
    {
    case DoExecute::CLEANUP:
        this->onCleanup();
        break;

    case DoExecute::CONFIGURE:
        this->onConfigure();
        break;

    case DoExecute::ACTIVATE:
        this->onActivating();
        break;

    case DoExecute::DEACTIVATE:
        this->onDeactivating();
        break;

    case DoExecute::SHUTDOWN:
        this->onShutdown();
        break;
    }

    _doExecute = DoExecute::NONE;

    // Publish current node status.
    lifecycle_msgs::NodeStatus msg;

    msg.lifecycle = static_cast<std::uint8_t>(_nodeState);
    msg.node_name = ros::this_node::getName();

    _pubState.publish(msg);
}

bool LifecycledNode::processServiceRequest(lifecycle_msgs::Lifecycle::Request& req, lifecycle_msgs::Lifecycle::Response& res)
{
    res.lifecycle = static_cast<std::uint8_t>(_nodeState);

    switch (req.action)
    {
        case lifecycle_msgs::Lifecycle::Request::ACTION_GET_STATE:
            return true;

        case lifecycle_msgs::Lifecycle::Request::ACTION_GO_IN_STATE:

            switch (static_cast<NodeStatus::State>(req.lifecycle))
            {
            case NodeStatus::State::UNCONFIGURED:
                if (_nodeState != NodeStatus::State::INACTIVE)
                    return false;

                _doExecute = DoExecute::CLEANUP;
                return true;

            case NodeStatus::State::INACTIVE:
                if (_nodeState == NodeStatus::State::UNCONFIGURED)
                    _doExecute = DoExecute::CONFIGURE;
                else if (_nodeState == NodeStatus::State::ACTIVE)
                    _doExecute = DoExecute::DEACTIVATE;
                else
                    return false;

                return true;

            case NodeStatus::State::ACTIVE:
                if (_nodeState != NodeStatus::State::INACTIVE)
                    return false;

                _doExecute = DoExecute::ACTIVATE;
                return true;

            case NodeStatus::State::FINALIZED:
                if (_nodeState != NodeStatus::State::UNCONFIGURED && _nodeState != NodeStatus::State::INACTIVE && _nodeState != NodeStatus::State::ACTIVE)
                    return false;

                _doExecute = DoExecute::SHUTDOWN;
                return true;

            default:
                return false;
            }

            return true;

        case lifecycle_msgs::Lifecycle::Request::ACTION_SHUTDOWN:
            this->onShutdown(); // Call immediately onShutdown(). Tested: the service call doesn't return. Need a better
                                // solution.
            return true;

        default:
            return false;
    }
}

} // end namespace lifecycled_node
