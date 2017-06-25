#ifndef ___NODE_STATE_EVENT_ACTOR_H___
#define ___NODE_STATE_EVENT_ACTOR_H___

#include <string>
#include <memory>

#include <lifecycle_msgs/CppNodeStatus.h>

namespace lifecycle_control {

class NodeStateEvent
{
public:

    enum class Event : std::uint8_t {
        UNDEFINED = 0,
        NEW_NODE,
        LIFECYCLE_CHANGED,
        NODE_TIMEOUT
    };

    NodeStateEvent(const Event event, const lifecycle_msgs::cpp::NodeStatus& nodeStatus)
        : _event(event),
          _nodeStatus(nodeStatus)
    {

    }

    inline Event event(void) const { return _event; }
    inline const lifecycle_msgs::cpp::NodeStatus& nodeStatus(void) const { return _nodeStatus; }

private:
    Event _event = Event::UNDEFINED;
    lifecycle_msgs::cpp::NodeStatus _nodeStatus;
};

class NodeStateEventActor : public std::enable_shared_from_this<NodeStateEventActor>
{
public:
    NodeStateEventActor(void) = default;
    virtual ~NodeStateEventActor(void) = default;

    virtual void nodeStateEvent(const NodeStateEvent& event) = 0;
};

} // end namespace lifecycle_control

#endif
