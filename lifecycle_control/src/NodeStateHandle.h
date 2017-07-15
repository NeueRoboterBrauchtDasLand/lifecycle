#ifndef ___NODE_STATE_HANDLE_H___
#define ___NODE_STATE_HANDLE_H___

#include <ros/ros.h>

#include <lifecycle_msgs/NodeStatus.h>

#include "NodeStateDatabase.h"
#include "NodeStateEventActor.h"

namespace lifecycle_control {

class NodeStateHandle
{
public:
    NodeStateHandle(std::shared_ptr<NodeStateDatabase>& database, ros::NodeHandle& nh);
    NodeStateHandle(const NodeStateHandle&) = delete;
    NodeStateHandle(NodeStateHandle&&) = delete;
    ~NodeStateHandle(void) = default;

    void registerEventActor(std::shared_ptr<NodeStateEventActor> actor);
    void nodeStatusCallback(const lifecycle_msgs::NodeStatus& msg);

    NodeStateHandle& operator =(const NodeStateHandle&) = delete;
    NodeStateHandle& operator =(NodeStateHandle&&) = delete;

private:
    void timerCallback(const ros::TimerEvent& event);

    std::shared_ptr<NodeStateDatabase> _stateDatabase;
    std::vector<std::shared_ptr<NodeStateEventActor>> _eventActors;
    ros::Timer _timer;
};

} // end namespace lifecycle_control

#endif
