#ifndef ___NODE_STATE_DATABASE_H___
#define ___NODE_STATE_DATABASE_H___

#include <map>
#include <vector>
#include <string>

#include <lifecycle_msgs/CppNodeStatus.h>

namespace lifecycle_control {

class NodeStateDatabase
{
public:
    NodeStateDatabase(const std::size_t maxSatesPerNode = 300);
    NodeStateDatabase(const NodeStateDatabase&) = delete;
    NodeStateDatabase(NodeStateDatabase&&) = default;
    ~NodeStateDatabase(void) = default;

    bool selectNode(const std::string& node);
    void releaseNode(void);
    void addNode(const std::string& node, const std::string& group = std::string());

    void addNodeState(const lifecycle_msgs::cpp::NodeStatus& state);
    lifecycle_msgs::cpp::NodeStatus getLastState(void) const;

    NodeStateDatabase& operator =(const NodeStateDatabase&) = delete;
    NodeStateDatabase& operator =(NodeStateDatabase&&) = default;

private:
    const std::size_t _maxStatesPerNode;

    std::map<std::string, std::size_t> _nodes;
    std::map<std::string, std::vector<std::size_t>> _groups;
    std::vector<std::shared_ptr<std::vector<lifecycle_msgs::cpp::NodeStatus>>> _states;
    std::vector<std::vector<lifecycle_msgs::cpp::NodeStatus>::iterator> _stateIts;
    std::vector<ros::Time> _lastStateStamp;
    std::size_t _selectedNodeIdx = 0;
    std::string _selectedNode;
};

} // end namespace lifecycle_control

#endif
