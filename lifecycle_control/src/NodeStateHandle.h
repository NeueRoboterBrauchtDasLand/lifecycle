#ifndef ___NODE_STATE_HANDLE_H___
#define ___NODE_STATE_HANDLE_H___


#include <lifecycle_msgs/NodeStatus.h>

#include "NodeStateDatabase.h"
#include "NodeStateEventActor.h"

namespace lifecycle_control {

class NodeStateHandle
{
public:
    NodeStateHandle(std::shared_ptr<NodeStateDatabase>& database);
    NodeStateHandle(const NodeStateHandle&) = delete;
    NodeStateHandle(NodeStateHandle&&) = default;
    ~NodeStateHandle(void) = default;

    void registerEventActor(std::shared_ptr<NodeStateEventActor>& actor);
    void nodeStatusCallback(const lifecycle_msgs::NodeStatus& msg);

    NodeStateHandle& operator =(const NodeStateHandle&) = delete;
    NodeStateHandle& operator =(NodeStateHandle&&) = default;

private:
    std::shared_ptr<NodeStateDatabase> _stateDatabase;
    std::vector<std::shared_ptr<NodeStateEventActor>> _eventActors;
};

} // end namespace lifecycle_control

#endif
