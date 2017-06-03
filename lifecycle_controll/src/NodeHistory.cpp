#include "NodeHistory.h"

namespace lifecycle_controll {

NodeHistory::NodeHistory(const std::size_t maxItems, const ros::Duration& statusLifeTime)
    : _maxStats(maxItems),
      _statLifeTime(statusLifeTime)
{

}

bool NodeHistory::insert(const lifecycle_msgs::cpp::NodeStatus& status)
{
    if (_inserts.size() + 1 >= _maxStats)
        return false;

    _inserts.push_front(status);

    return true;
}

void NodeHistory::cleanup(void)
{
    _stats.clear();
    _inserts.clear();
}

void NodeHistory::update(void)
{
    if (!_inserts.size())
        return;

    if (_stats.size() + _inserts.size() > _maxStats)
    {
        const int diff = (_stats.size() + _inserts.size()) - _maxStats;
        auto start = _stats.end();

        std::advance(start, -diff);
        _stats.erase(start, _stats.end());
    }

    if (_inserts.size())
    {
        _stats.insert(_stats.cbegin(), _inserts.cbegin(), _inserts.cend());
        _inserts.clear();
    }

    if (!_stats.size())
        return;

    const ros::Time lastStamp(_stats.front().stamp());
    auto delRange = _stats.begin();


    while (delRange != _stats.end())
    {
        if (lastStamp - delRange->stamp() > _statLifeTime)
            break;

        ++delRange;
    }

    if (delRange != _stats.cbegin() && delRange != _stats.cend())
        _stats.erase(delRange, _stats.cend());
}

} // end namespace lifecycle_controll
