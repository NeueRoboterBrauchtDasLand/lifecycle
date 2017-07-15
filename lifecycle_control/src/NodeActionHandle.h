#ifndef ___NODE_ACTION_HANDLE_H___
#define ___NODE_ACTION_HANDLE_H___

#include <map>
#include <vector>

#include <ros/ros.h>

#include "NodeAction.h"
#include "NodeStateEventActor.h"

namespace lifecycle_control {

class NodeActionHandle : public NodeStateEventActor
{
public:
    NodeActionHandle(std::shared_ptr<ros::NodeHandle>& nh);
    NodeActionHandle(const NodeActionHandle&) = default;
    NodeActionHandle(NodeActionHandle&&) = default;
    ~NodeActionHandle(void) = default;

    bool createAction(const std::string& node, const lifecycle_msgs::cpp::NodeStatus::State targetLifecycle);
    std::vector<NodeAction> allActions(void) const;

    virtual void nodeStateEvent(const NodeStateEvent& event);

    NodeActionHandle& operator =(const NodeActionHandle&) = default;
    NodeActionHandle& operator =(NodeActionHandle&&) = default;

private:
    void callbackTimer(const ros::TimerEvent& event);

    std::shared_ptr<ros::NodeHandle> _nh;
    ros::Timer _timer;
    std::map<std::string, NodeAction> _actions;
    std::map<std::string, std::shared_ptr<ros::ServiceClient>> _srvsNodeAction;
};

} // end namespace lifecycle_control

#endif
