#include "NodeActionHandle.h"

#include <lifecycle_msgs/Lifecycle.h>

namespace lifecycle_control {

NodeActionHandle::NodeActionHandle(std::shared_ptr<ros::NodeHandle>& nh)
    : _nh(nh),
      _timer(_nh->createTimer(ros::Duration(0.5), &NodeActionHandle::callbackTimer, this))
{

}

bool NodeActionHandle::createAction(const std::string& node, const lifecycle_msgs::cpp::NodeStatus::State targetLifecycle)
{
    const auto actionIt = _actions.find(node);

    if (actionIt == _actions.end())
    // Inser the new node and create an action.
    {
        _actions.insert(std::pair<std::string, NodeAction>(node, NodeAction(_srvsNodeAction[node], node, targetLifecycle)));
        return true;
    }

    if (actionIt->second.isExecuting())
    // An action is still processing.
    {
        ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << ": the node '" << node
                         << "' has an active action. Can't create an additional action. --> return.");
        return false;
    }

    // Override the old action with a new created one.
    actionIt->second = NodeAction{_srvsNodeAction[node], node, targetLifecycle};

    if (actionIt->second.error() != NodeAction::Error::NONE)
    {
        ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << ": the node '" << node
                         << "' rejected the lifecycle change request. --> return.");
        return false;
    }

    return true;
}

std::vector<NodeAction> NodeActionHandle::allActions(void) const
{
    std::vector<NodeAction> actions(_actions.size());
    auto actionIt = actions.begin();

    for (const auto& actionMapIt : _actions)
    {
        *actionIt = actionMapIt.second;
        ++actionIt;
    }

    return actions;
}

void NodeActionHandle::nodeStateEvent(const NodeStateEvent& event)
{
    switch (event.event())
    {
    case NodeStateEvent::Event::NEW_NODE:
        {
            std::shared_ptr<ros::ServiceClient> client = std::make_shared<ros::ServiceClient>();
            *client = _nh->serviceClient<lifecycle_msgs::Lifecycle>("/lifecycle/service/" + event.nodeStatus().name());
            _srvsNodeAction.insert(std::pair<std::string, std::shared_ptr<ros::ServiceClient>>(event.nodeStatus().name(),
                                                                                               client));
        }
        break;

    case NodeStateEvent::Event::LIFECYCLE_CHANGED:
        {
            auto actionIt = _actions.find(event.nodeStatus().name());

            if (actionIt == _actions.end())
            // No active action of this node. --> Do nothing...
                break;

            actionIt->second.process(event.nodeStatus().lifecycle());
        }
        break;

    default:
        ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << ": node state event is not implemented. --> return");
        return;
    }
}

void NodeActionHandle::callbackTimer(const ros::TimerEvent& event)
{
    for (auto& action : _actions)
        action.second.process();
}

} // end namespace lifecycle_control
