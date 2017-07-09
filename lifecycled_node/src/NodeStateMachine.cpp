#include "lifecycled_node/NodeStateMachine.h"

namespace lifecycled_node {

NodeStateMachine::NodeStateMachine(const State initialState)
    : _currentState(initialState)
{

}

bool NodeStateMachine::canChangeTo(const State to) const
{
    switch (to)
    {
    case State::UNCONFIGURED:
        return _currentState == State::INACTIVE;

    case State::INACTIVE:
        return _currentState == State::UNCONFIGURED || _currentState == State::ACTIVE;

    case State::ACTIVE:
        return _currentState == State::INACTIVE;

    case State::FINALIZED:
        return _currentState == State::UNCONFIGURED || _currentState == State::INACTIVE ||
               _currentState == State::ACTIVE;

    default:
        return false;
    }
}

bool NodeStateMachine::changeTo(const State to)
{
    if (!this->canChangeTo(to))
        return false;

    _currentState = to;
    return true;
}

} // end namespace lifecycled_node
