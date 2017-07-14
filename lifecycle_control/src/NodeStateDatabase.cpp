#include "NodeStateDatabase.h"

#include <ros/ros.h>

namespace lifecycle_control {

NodeStateDatabase::NodeStateDatabase(const std::size_t maxSatesPerNode)
    : _maxStatesPerNode(maxSatesPerNode)
{

}

bool NodeStateDatabase::selectNode(const std::string& node)
{
    const auto index = _nodes.find(node);

    if (index == _nodes.end())
    // No entry of this node in the database.
        return false;

    _selectedNodeIdx = index->second;
    _selectedNode    = node;

    return true;
}

bool NodeStateDatabase::existsGroup(const std::string& group) const
{
    return _groups.find(group) != _groups.end();
}

void NodeStateDatabase::releaseNode(void)
{
    _selectedNodeIdx = 0;
    _selectedNode.clear();
}

void NodeStateDatabase::addNode(const std::string& node, const std::string& group)
{
    // Add a new element to the index container.
    _nodes.insert(std::pair<std::string, std::size_t>(node, _nodes.size()));

    // Create a node status container of the size _maxStatesPerNode.
    _states.push_back(std::make_shared<std::vector<lifecycle_msgs::cpp::NodeStatus>>(_maxStatesPerNode));

    // Add an iterator for the node states.
    _stateIts.push_back(_states.back()->begin());

    // Add new time stamp element to _lastStateStamp container.
    _lastStateStamp.push_back(ros::Time());

    if (group.empty())
        return;

    // Check if group of this node exists.
    auto groupIt = _groups.find(group);

    if (groupIt == _groups.end())
    // Group is new. Add it to the group container.
    {
        std::vector<std::size_t> indices;
        indices.push_back(_nodes[node]);
        _groups.insert(std::pair<std::string, std::vector<std::size_t>>(group, indices));
    }
    else
    // Group is known. Add node index to the indices container of the group.
    {
        groupIt->second.push_back(_nodes[node]);
    }
}

void NodeStateDatabase::addNodeState(const lifecycle_msgs::cpp::NodeStatus& state)
{
    if (_selectedNode.empty())
    {
        ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << ": no node is selected! --> return");
        return;
    }

    auto& stateIt = _stateIts[_selectedNodeIdx];

    // Increment the iterator and overwrite the old state.
    if (++stateIt >= _states[_selectedNodeIdx]->end())
         stateIt = _states[_selectedNodeIdx]->begin();

    *stateIt = state;
    _lastStateStamp[_selectedNodeIdx] = state.stamp();
}

lifecycle_msgs::cpp::NodeStatus NodeStateDatabase::getLastState(void) const
{
    if (_selectedNode.empty())
    {
        ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << ": no node is selected! --> return");
        return lifecycle_msgs::cpp::NodeStatus{};
    }

    return *_stateIts[_selectedNodeIdx];
}

ros::Time NodeStateDatabase::getLastStateStamp(void) const
{
    if (_selectedNode.empty())
    {
        ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << ": no node is selected! --> return");
        return ros::Time{};
    }

    return _lastStateStamp[_selectedNodeIdx];
}

std::vector<lifecycle_msgs::cpp::NodeStatus> NodeStateDatabase::getLastStateOfPendingNodes(const ros::Duration& timeout) const
{
    std::vector<lifecycle_msgs::cpp::NodeStatus> states;
    const ros::Time currentTime(ros::Time::now());

    for (unsigned int node = 0; node < _lastStateStamp.size(); ++node)
        if (!_lastStateStamp[node].isZero() && currentTime - _lastStateStamp[node] > timeout)
            states.push_back(*_stateIts[node]);

    return states;
}

std::vector<lifecycle_msgs::cpp::NodeStatus> NodeStateDatabase::getLastStateOfNodes(void) const
{
    std::vector<lifecycle_msgs::cpp::NodeStatus> states(_nodes.size());

    for (unsigned int node = 0; node < _stateIts.size(); ++node)
        states[node] = *_stateIts[node];

    return states;
}

std::vector<std::string> NodeStateDatabase::getNodes(void) const
{
    std::vector<std::string> nodes(_nodes.size());
    auto nodesIt(nodes.begin());

    for (const auto& node : _nodes)
    {
        *nodesIt = node.first;
        ++nodesIt;
    }

    return nodes;
}

std::vector<std::string> NodeStateDatabase::getNodes(const std::string& group) const
{
    const auto groupIt(_groups.find(group));

    if (groupIt == _groups.end())
    {
        ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << ": group is unkown. --> return empty container.");
        return {};
    }

    const auto indices(groupIt->second);
    std::vector<std::string> nodes(indices.size());

    for (unsigned int i = 0; i < indices.size(); ++i)
        nodes[i] = _stateIts[indices[i]]->name();

    return nodes;
}

std::vector<std::string> NodeStateDatabase::getGroups(void) const
{
    std::vector<std::string> groups(_groups.size());
    auto _groupsIt(_groups.begin());

    for (auto& group : groups)
    {
        group = _groupsIt->first;
        ++_groupsIt;
    }

    return groups;
}

} // end namespace lifecycle_control
