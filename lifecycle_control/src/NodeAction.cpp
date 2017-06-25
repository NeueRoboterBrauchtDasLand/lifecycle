#include "NodeAction.h"

#include <lifecycle_msgs/Lifecycle.h>

namespace lifecycle_control {

ros::Duration NodeAction::_timeoutValue = ros::Duration(10.0);

NodeAction::NodeAction(std::shared_ptr<ros::ServiceClient> srvClien,
                       const std::string& nodeName,
                       const lifecycle_msgs::cpp::NodeStatus::State targetLifecycle)
    : _node(nodeName),
      _srvClient(srvClien),
      _targetLifecycle(targetLifecycle)
{
    this->callLifecycleService();
}

void NodeAction::process(void)
{
    if (!_executing)
        return;

    if (ros::Time::now() - _stamp > _timeoutValue)
    {
        _error = Error::TIMEOUT;
        _executing = false;
    }
}

void NodeAction::process(const lifecycle_msgs::cpp::NodeStatus::State newLifecycle)
{
    if (!_executing)
        return;

    if (newLifecycle != _targetLifecycle)
        _error = Error::WRONG_LIFECYCLE;

    _executing = false;
}

void NodeAction::callLifecycleService(void)
{
    lifecycle_msgs::Lifecycle msg;

    msg.request.action = lifecycle_msgs::Lifecycle::Request::ACTION_GO_IN_STATE;
    msg.request.lifecycle = static_cast<std::uint8_t>(_targetLifecycle);
    _stamp = ros::Time::now();

    if (!_srvClient->call(msg))
    {
        _error = Error::REJECTED;
        _executing = false;
    }
    else
    {
        _error = Error::NONE;
        _executing = true;
    }
}

} // end namespace lifecycle_control
