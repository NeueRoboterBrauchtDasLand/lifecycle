#ifndef ___NODE_STATE_HANDLE_H___
#define ___NODE_STATE_HANDLE_H___

#include <map>
#include <vector>
#include <string>

#include <lifecycle_msgs/NodeStatus.h>

#include "NodeStateEventActor.h"

namespace lifecycle_control {

class NodeStateHandle
{
public:
    NodeStateHandle(void) = default;
    NodeStateHandle(const NodeStateHandle&) = delete;
    NodeStateHandle(NodeStateHandle&&) = default;
    ~NodeStateHandle(void) = default;

    void registerEventActor(std::shared_ptr<NodeStateEventActor>& actor);
    void nodeStatusCallback(const lifecycle_msgs::NodeStatus& msg);

    NodeStateHandle& operator =(const NodeStateHandle&) = delete;
    NodeStateHandle& operator =(NodeStateHandle&&) = default;

private:
    std::map<std::string, std::size_t> _nodes;
    std::map<std::string, std::vector<std::size_t>> _groups;
    std::vector<lifecycle_msgs::cpp::NodeStatus> _lastState;
    std::vector<ros::Time> _lastStateStamp;

    std::vector<std::shared_ptr<NodeStateEventActor>> _eventActors;
};

} // end namespace lifecycle_control

#endif
