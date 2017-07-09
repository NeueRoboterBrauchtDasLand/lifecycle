#ifndef ___LIFECYCLED_NODE_NODE_STATE_MACHINE_H___
#define ___LIFECYCLED_NODE_NODE_STATE_MACHINE_H___

#include <lifecycle_msgs/cpp/NodeStatus.h>

namespace lifecycled_node {

class NodeStateMachine
{
using State = lifecycle_msgs::cpp::NodeStatus::State;

public:
    NodeStateMachine(const State initialState = State::CREATED);
    NodeStateMachine(const NodeStateMachine&) = default;
    NodeStateMachine(NodeStateMachine&&) = default;
    ~NodeStateMachine(void) = default;

    bool canChangeTo(const State to) const;
    bool changeTo(const State to);
    inline State currentState(void) const { return _currentState; }

    NodeStateMachine& operator =(const NodeStateMachine&) = default;
    NodeStateMachine& operator =(NodeStateMachine&&) = default;

private:
    State _currentState;
};

} // end namespace lifecycled_node

#endif
