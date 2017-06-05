#ifndef ___NODE_ACTION_HANDLE_H___
#define ___NODE_ACTION_HANDLE_H___

#include <map>

#include "NodeAction.h"

namespace lifecycle_control {

class NodeActionHandle
{
public:
    NodeActionHandle(void) = default;
    NodeActionHandle(const NodeActionHandle&) = default;
    NodeActionHandle(NodeActionHandle&&) = default;
    ~NodeActionHandle(void) = default;

    bool createAction(const std::string& node, const lifecycle_msgs::cpp::NodeStatus::State targetLifecycle);
    void updateNodeStates(const std::string& node, const lifecycle_msgs::cpp::NodeStatus::State lifecycle);

    NodeActionHandle& operator =(const NodeActionHandle&) = default;
    NodeActionHandle& operator =(NodeActionHandle&&) = default;

private:
    std::map<std::string, NodeAction> _actions;
};

} // end namespace lifecycle_control

#endif
