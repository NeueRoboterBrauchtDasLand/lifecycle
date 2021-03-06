#ifndef ___NODE_ACTION_H___
#define ___NODE_ACTION_H___

#include <ros/ros.h>

#include <lifecycle_msgs/cpp/NodeStatus.h>
#include <lifecycle_msgs/NodeAction.h>

namespace lifecycle_control {

class NodeAction
{
public:

    enum class Error : std::uint8_t {
        NONE = 0,
        REJECTED,
        TIMEOUT,
        WRONG_LIFECYCLE,
    };

    static std::string errorName(const Error error)
    {
        switch (error)
        {
        case Error::NONE:
            return { "NONE" };
        case Error::REJECTED:
            return { "REJECTED" };
        case Error::TIMEOUT:
            return { "TIMEOUT" };
        case Error::WRONG_LIFECYCLE:
            return { "WRONG_LIFECYCLE" };
        default:
            return { "UNKNOWN" };
        }
    }

    NodeAction(void) = default;
    NodeAction(std::shared_ptr<ros::ServiceClient> srvClien,
               const std::string& nodeName,
               const lifecycle_msgs::cpp::NodeStatus::State targetLifecycle);
    NodeAction(const lifecycle_msgs::NodeAction& msg);

    NodeAction(const NodeAction&) = default;
    NodeAction(NodeAction&&) = default;
    ~NodeAction(void) = default;

    void process(void);
    void process(const lifecycle_msgs::cpp::NodeStatus::State newLifecycle);

    inline Error error(void) const { return _error; }
    inline bool isExecuting(void) const { return _executing; }

    lifecycle_msgs::NodeAction toMsg(void) const;

    NodeAction& operator =(const NodeAction&) = default;
    NodeAction& operator =(NodeAction&&) = default;

private:
    void callLifecycleService(void);

    std::string _node;
    std::shared_ptr<ros::ServiceClient> _srvClient;
    lifecycle_msgs::cpp::NodeStatus::State _targetLifecycle = lifecycle_msgs::cpp::NodeStatus::State::UNDEFINED;
    ros::Time _stamp;
    bool _executing = false;
    Error _error = Error::NONE;

    static ros::Duration _timeoutValue;
};

} // end namespace lifecycle_control

#endif
