#include "HelloRosPublisherNode.h"

#include <iostream>

#include <std_msgs/String.h>

namespace lifecycled_node {

namespace test {

HelloRosPublisherNode::HelloRosPublisherNode(ros::NodeHandle& privNh, ros::NodeHandle& nh)
    : LifecycledNode(privNh, nh),
      _nh(nh)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    privNh.param<double>("publish_freq", _publishFreq, 1.0);
}

HelloRosPublisherNode::~HelloRosPublisherNode(void)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
}

void HelloRosPublisherNode::cleanup(void)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    _pubHelloRos.shutdown();
}

void HelloRosPublisherNode::configure(void)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    _pubHelloRos = _nh.advertise<std_msgs::String>("message", 1);
}

void HelloRosPublisherNode::activating(void)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    _timer = _nh.createTimer(ros::Duration(_publishFreq), &HelloRosPublisherNode::callbackTimer, this);
}

void HelloRosPublisherNode::deactivating(void)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    _timer.stop();
}

void HelloRosPublisherNode::shuttingDown(void)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    ros::shutdown();
}

void HelloRosPublisherNode::callbackTimer(const ros::TimerEvent& event)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    std_msgs::String msg;

    msg.data = "Hello ROS!";

    _pubHelloRos.publish(msg);
}

void HelloRosPublisherNode::exec(void)
{
    ros::spin();
}

} // end namespace test

} // end namespace lifecycled_node

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hello_ros_publisher");
    ros::NodeHandle privNh("~"), nh;
    lifecycled_node::test::HelloRosPublisherNode node(privNh, nh);

    node.exec();
}
