#ifndef ___NODE_HISTORY_H___
#define ___NODE_HISTORY_H___

#include <list>
#include <cstddef>

#include <lifecycled_node/NodeStatus.h>

namespace lifecycle_controll {

class NodeHistory
{
public:
    NodeHistory(const std::size_t maxItems = 1000, const ros::Duration& statusLifeTime = ros::Duration(120.0));

    bool insert(const lifecycled_node::NodeStatus& status);
    void cleanup(void);
    void update(void);

    void setMaxItems(const std::size_t max) { _maxStats = max; }
    void setStatusLifeTime(const ros::Duration& lifeTime) { _statLifeTime = lifeTime; }

    unsigned int count(void) const { return _stats.size(); }

private:
    std::list<lifecycled_node::NodeStatus> _stats;
    std::list<lifecycled_node::NodeStatus> _inserts;
    std::size_t _maxStats;
    ros::Duration _statLifeTime;
};

} // end namespace lifecycle controll

inline std::ostream& operator<< (std::ostream& os, const lifecycle_controll::NodeHistory& history)
{
    os << "Node History:" << std::endl;
    os << "--------------------------------------------------------------------------------------" << std::endl;

    return os;
}

#endif
