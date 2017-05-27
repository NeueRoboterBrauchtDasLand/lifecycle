#include <map>
#include <string>

#include <ros/ros.h>

#include "NodeHistory.h"

std::map<std::string, lifecycle_controll::NodeHistory> _histories;

void nodeStatusCallback(const lifecycle_msgs::NodeStatus& msg)
{
    auto item = _histories.find(msg.node_name);

    if (item == _histories.end())
    {
    	auto newItem(std::pair<std::string, lifecycle_controll::NodeHistory>(msg.node_name,
                                                                             lifecycle_controll::NodeHistory()));
        _histories.insert(newItem);
        _histories[msg.node_name].insert(lifecycled_node::NodeStatus(msg));
    }
    else
    {
        item->second.insert(lifecycled_node::NodeStatus(msg));
    }
}

void printAllNodeNames(void)
{
    for (const auto& node : _histories)
        std::cout << node.first << std::endl;
}

void updateAllHistories(void)
{
    for (auto& node : _histories)
        node.second.update();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lifecycle_controller");
    ros::NodeHandle nh;
    ros::Subscriber subStatus(nh.subscribe("/lifecycle/status", 100, nodeStatusCallback));

    ros::Rate rate(1.0);

    while (ros::ok())
    {
        updateAllHistories();
        printAllNodeNames();

        ros::spinOnce();
        rate.sleep();
    }
}
