#ifndef ___NODE_STATUS_H___
#define ___NODE_STATUS_H___

#include <lifecycle_msgs/NodeStatus.h>

namespace lifecycle_msgs {

namespace cpp {

class NodeStatus
{
public:

    enum class State : std::uint8_t {
        CREATED = 0,
        UNCONFIGURED,
        INACTIVE,
        ACTIVE,
        FINALIZED,
    	UNDEFINED
    };

    NodeStatus(void) = default;
    NodeStatus(const ::lifecycle_msgs::NodeStatus& msg)
        : _lifecycle(static_cast<State>(msg.lifecycle)),
          _name(msg.node_name),
          _stamp(msg.stamp)
    {

    }
    NodeStatus(const NodeStatus&) = default;
    NodeStatus(NodeStatus&&) = default;
    ~NodeStatus(void) = default;

    const std::string& name(void) const { return _name; }
    State lifecycle(void) const { return _lifecycle; }
    const ros::Time stamp(void) const { return _stamp; }
    bool isValid(void) const { return !_name.empty(); }

    NodeStatus& operator =(const NodeStatus&) = default;
    NodeStatus& operator =(NodeStatus&&) = default;
    ::lifecycle_msgs::NodeStatus toMsg(void) const
    {
        ::lifecycle_msgs::NodeStatus msg;

        msg.node_name = _name;
        msg.stamp     = _stamp;
        msg.lifecycle = static_cast<std::uint8_t>(_lifecycle);

        return msg;
    }

private:
    State _lifecycle = State::UNDEFINED;
    std::string _name;
    ros::Time _stamp;
};

} // end namespace cpp

} // end namespace lifecycle_msgs

#endif
