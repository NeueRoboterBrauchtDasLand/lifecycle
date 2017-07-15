#include "lifecycled_node/LifecycledNode.h"

#include <iostream>

namespace lifecycled_node {

inline void WARN(const std::string& msg)
{
    ROS_WARN_STREAM(ros::this_node::getName() + msg);
}

inline void ERROR(const std::string& msg)
{
    ROS_ERROR_STREAM(ros::this_node::getName() + ": " + msg);
}

LifecycledNode::LifecycledNode(void)
{
    if (!_stateMachine.changeTo(NodeStatus::State::CREATED))
        ERROR("Can't change to lifecycle state CREATED. This must never happen! BUG!!!");
}

LifecycledNode::LifecycledNode(ros::NodeHandle& privNh, ros::NodeHandle& nh)
{
    if (!_stateMachine.changeTo(NodeStatus::State::CREATED))
        ERROR("Can't change to lifecycle state CREATED. This must never happen! BUG!!!");
    this->initializeLifecycle(privNh, nh);
}

LifecycledNode::~LifecycledNode(void)
{
    _timer.stop();
}

void LifecycledNode::initializeLifecycle(ros::NodeHandle& privNh, ros::NodeHandle& nh)
{
    if (_stateMachine.currentState() != NodeStatus::State::CREATED)
    {
        WARN(" is already initialized. --> Do nothing and return.");
        return;
    }

    privNh.param<double>("lifecycle_processing_freq", _processingFreq, _processingFreq);
    privNh.param<std::string>("node_group", _nodeGroup, _nodeGroup);

    _pubState = nh.advertise<lifecycle_msgs::NodeStatus>("/lifecycle/status", 1);
    _srvResponser = nh.advertiseService("/lifecycle/service/" + ros::this_node::getName(),
                                        &LifecycledNode::processServiceRequest,
                                        this);
    _timer = nh.createTimer(ros::Rate(_processingFreq), &LifecycledNode::processLifecycle, this);

    if (!_stateMachine.changeTo(NodeStatus::State::UNCONFIGURED))
        ERROR("Can't change to lifecycle state UNCONFIGURED. This must never happen! BUG!!!");
}

void LifecycledNode::onCleanup(void)
{
    this->cleanup();

    if (!_stateMachine.changeTo(NodeStatus::State::UNCONFIGURED))
        ERROR("Can't change to lifecycle state UNCONFIGURED. This must never happen! BUG!!!");
}

void LifecycledNode::onConfigure(void)
{
    this->configure();

    if (!_stateMachine.changeTo(NodeStatus::State::INACTIVE))
        ERROR("Can't change to lifecycle state INACTIVE. This must never happen! BUG!!!");
}

void LifecycledNode::onActivating(void)
{
    this->activating();

    if (!_stateMachine.changeTo(NodeStatus::State::ACTIVE))
        ERROR("Can't change to lifecycle state ACTIVE. This must never happen! BUG!!!");
}

void LifecycledNode::onDeactivating(void)
{
    this->deactivating();

    if (!_stateMachine.changeTo(NodeStatus::State::INACTIVE))
        ERROR("Can't change to lifecycle state INACTIVE. This must never happen! BUG!!!");
}

void LifecycledNode::onShutdown(void)
{
    this->shuttingDown();

    if (!_stateMachine.changeTo(NodeStatus::State::FINALIZED))
        ERROR("Can't change to lifecycle state FINALIZED. This must never happen! BUG!!!");
}

void LifecycledNode::processLifecycle(const ros::TimerEvent& event)
{
    // TODO: maybe a console output would be nice here (changed from ... to ...).
    // TODO: remove the on methods and put the virtual function including the switch case to a new changeLifecycle function.
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

    msg.lifecycle = static_cast<std::uint8_t>(_stateMachine.currentState());
    msg.node_name = ros::this_node::getName();
    msg.group     = _nodeGroup;

    _pubState.publish(msg);
}

bool LifecycledNode::processServiceRequest(lifecycle_msgs::Lifecycle::Request& req, lifecycle_msgs::Lifecycle::Response& res)
{
    res.lifecycle = static_cast<std::uint8_t>(_stateMachine.currentState());

    switch (req.action)
    {
        case lifecycle_msgs::Lifecycle::Request::ACTION_GET_STATE:
            return true;

        case lifecycle_msgs::Lifecycle::Request::ACTION_GO_IN_STATE:

            switch (static_cast<NodeStatus::State>(req.lifecycle))
            {
            case NodeStatus::State::UNCONFIGURED:
                if (!_stateMachine.canChangeTo(NodeStatus::State::UNCONFIGURED))
                {
                    WARN("Can't change in lifecycle state '" + NodeStatus::stateName(NodeStatus::State::UNCONFIGURED));
                    return false;
                }

                _doExecute = DoExecute::CLEANUP;
                return true;

            case NodeStatus::State::INACTIVE:
                if (!_stateMachine.canChangeTo(NodeStatus::State::INACTIVE))
                {
                    WARN("Can't change in lifecycle state '" + NodeStatus::stateName(NodeStatus::State::INACTIVE));
                    return false;
                }

                if (_stateMachine.currentState() == NodeStatus::State::UNCONFIGURED)
                    _doExecute = DoExecute::CONFIGURE;
                else if (_stateMachine.currentState() == NodeStatus::State::ACTIVE)
                    _doExecute = DoExecute::DEACTIVATE;
                else
                    return false;

                return true;

            case NodeStatus::State::ACTIVE:
                if (!_stateMachine.canChangeTo(NodeStatus::State::ACTIVE))
                {
                    WARN("Can't change in lifecycle state '" + NodeStatus::stateName(NodeStatus::State::ACTIVE));
                    return false;
                }

                _doExecute = DoExecute::ACTIVATE;
                return true;

            case NodeStatus::State::FINALIZED:
                if (!_stateMachine.canChangeTo(NodeStatus::State::FINALIZED))
                {
                    WARN("Can't change in lifecycle state '" + NodeStatus::stateName(NodeStatus::State::FINALIZED));
                    return false;
                }

                _doExecute = DoExecute::SHUTDOWN;
                return true;

            default:
                return false;
            }

            return true;

        case lifecycle_msgs::Lifecycle::Request::ACTION_SHUTDOWN:
            this->onShutdown(); // Call immediately onShutdown(). Tested: the service call doesn't return. TODO: Need a
                                // better solution.
            return true;

        default:
            return false;
    }
}

} // end namespace lifecycled_node
