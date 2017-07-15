#include "NodeStateHandle.h"

#include <algorithm>

#include <ros/ros.h>

namespace lifecycle_control {

NodeStateHandle::NodeStateHandle(std::shared_ptr<NodeStateDatabase>& database, ros::NodeHandle& nh)
    : _stateDatabase(database),
      _timer(nh.createTimer(ros::Duration(0.1), &NodeStateHandle::timerCallback, this))
{

}

void NodeStateHandle::registerEventActor(std::shared_ptr<NodeStateEventActor> actor)
{
    // Check if the actor is already registered.
    if (std::find(_eventActors.begin(), _eventActors.end(), actor) != _eventActors.end())
    {
        ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << ": this event actor is already registered. --> return");
        return;
    }

    _eventActors.push_back(actor);
}

void NodeStateHandle::nodeStatusCallback(const lifecycle_msgs::NodeStatus& msg)
{
    // Check if a database is set.
    if (!_stateDatabase)
    {
        ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << ": no database is set. Can't add new state. --> return");
        return;
    }

    NodeStateEvent::Event event = NodeStateEvent::Event::UNDEFINED;

    // Select the node.
    if (!_stateDatabase->selectNode(msg.node_name))
    {
        _stateDatabase->addNode(msg.node_name, msg.group);
        event = NodeStateEvent::Event::NEW_NODE;

        if (!_stateDatabase->selectNode(msg.node_name))
        {
            ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << ": added new node to database, but can't select this node. --> return");
            return;
        }
    }

    lifecycle_msgs::cpp::NodeStatus state(msg);

    // If event == UNDEFINED then the node is kown.
    if (event == NodeStateEvent::Event::UNDEFINED)
    	// Check if the lifecycle of the node was changed.
        if (_stateDatabase->getLastState().lifecycle() != state.lifecycle())
            event = NodeStateEvent::Event::LIFECYCLE_CHANGED;

    // Add new state of the node to the database.
    _stateDatabase->addNodeState(state);

    if (event != NodeStateEvent::Event::UNDEFINED)
    // Call all event actors until one has accepted this event.
    {
        const NodeStateEvent nodeStateEvent(event, state);

        for (auto& actor : _eventActors)
            actor->nodeStateEvent(nodeStateEvent);
    }
}

void NodeStateHandle::timerCallback(const ros::TimerEvent& event)
{
    const auto pendingNodes(_stateDatabase->getLastStateOfPendingNodes(ros::Duration(10.0)));

    if (pendingNodes.size())
    {
        for (const auto& node : pendingNodes)
        {
            const NodeStateEvent nodeStateEvent(NodeStateEvent::Event::NODE_TIMEOUT, node);

            for (auto& actor : _eventActors)
                actor->nodeStateEvent(nodeStateEvent);
        }
    }
}

} // end namespace lifecycle_control
