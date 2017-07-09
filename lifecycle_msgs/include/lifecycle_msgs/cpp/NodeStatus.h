#ifndef ___CPP_NODE_STATUS_H___
#define ___CPP_NODE_STATUS_H___

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
        : _stamp(msg.stamp),
          _name(msg.node_name),
	      _group(msg.group),
          _lifecycle(static_cast<State>(msg.lifecycle))
    {

    }
    NodeStatus(const NodeStatus&) = default;
    NodeStatus(NodeStatus&&) = default;
    ~NodeStatus(void) = default;

    const std::string& name(void) const { return _name; }
    const std::string& group(void) const { return _group; }
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
        msg.group     = _group;

        return msg;
    }

    static std::string stateName(const State state)
    {
        switch (state)
        {
        case State::CREATED:
            return { "CREATED" };
        case State::UNCONFIGURED:
            return { "UNCONFIGURED" };
        case State::INACTIVE:
            return { "INACTIVE" };
        case State::ACTIVE:
            return { "ACTIVE" };
        case State::FINALIZED:
            return { "FINALIZED" };
        case State::UNDEFINED:
            return { "UNDEFINED" };
        default:
            return { "UNKOWN" };
        }
    }

private:
    ros::Time _stamp;
    std::string _name;
    std::string _group;
    State _lifecycle = State::UNDEFINED;
};

} // end namespace cpp

} // end namespace lifecycle_msgs

#endif
