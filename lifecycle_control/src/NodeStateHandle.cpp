#include "NodeStateHandle.h"

namespace lifecycle_control {

void NodeStateHandle::registerEventActor(std::shared_ptr<NodeStateEventActor>& actor)
{

}

void NodeStateHandle::nodeStatusCallback(const lifecycle_msgs::NodeStatus& msg)
{
    auto index = _nodes.find(msg.node_name);

    // Node name is new.
    if (index == _nodes.end())
    {
        // Add a new element to all container and insert the current status.
        _nodes.insert(std::pair<std::string, std::size_t>(msg.node_name, _histories.size()));

	// TODO: move history to ...
//        _histories.push_back(std::make_shared<lifecycle_controll::NodeHistory>());
//        _histories.back()->insert(lifecycle_msgs::cpp::NodeStatus(msg));
        _lastStateStamp.push_back(msg.stamp);

        // Create service client for the new node. TODO: move services below to action handle.
//        _srvsNodeAction.push_back(std::make_shared<ros::ServiceClient>());
//        *(_srvsNodeAction.back()) = _nh->serviceClient<lifecycle_msgs::Lifecycle>("/lifecycle/service/" + msg.node_name);

        // TODO: create new node event.

        // Check if the group is also new.
        auto group = _groups.find(msg.group);

        // Group is new. Add it to the group container.
        if (group == _groups.end())
        {
            std::vector<std::size_t> indices;
            indices.push_back(_nodes[msg.node_name]);
            _groups.insert(std::pair<std::string, std::vector<std::size_t>>(msg.group, indices));
            // TODO: create new group event.
        }
        // Group is known. Add node index to the indices container of the group.
        else
        {
            group->second.push_back(_nodes[msg.node_name]);
        }
    }
    // Node name is known. Insert status to history and update lastStatus.
    else
    {
//        _histories[index->second]->insert(lifecycle_msgs::cpp::NodeStatus(msg)); TODO: move history.
        _lastStateStamp[index->second] = msg.stamp;
    }
}

} // end namespace lifecycle_control
