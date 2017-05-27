#include "HelloRosPublisherNode.h"

#include <std_msgs/String.h>

namespace lifecycled_node {

namespace test {

HelloRosPublisherNode::HelloRosPublisherNode(ros::NodeHandle& privNh, ros::NodeHandle& nh)
    : LifecycledNode(privNh, nh),
      _nh(nh)
{
    ROS_INFO_STREAM(ros::this_node::getName() + __PRETTY_FUNCTION__);
    privNh.param<double>("publish_freq", _publishFreq, 1.0);
}

HelloRosPublisherNode::~HelloRosPublisherNode(void)
{
    ROS_INFO_STREAM(ros::this_node::getName() + __PRETTY_FUNCTION__);
}

void HelloRosPublisherNode::cleanup(void)
{
    ROS_INFO_STREAM(ros::this_node::getName() + __PRETTY_FUNCTION__);
    _pubHelloRos.shutdown();
}

void HelloRosPublisherNode::configure(void)
{
    ROS_INFO_STREAM(ros::this_node::getName() + __PRETTY_FUNCTION__);
    _pubHelloRos = _nh.advertise<std_msgs::String>("message", 1);
}

void HelloRosPublisherNode::activating(void)
{
    ROS_INFO_STREAM(ros::this_node::getName() + __PRETTY_FUNCTION__);
    _timer = _nh.createTimer(ros::Rate(_publishFreq), &HelloRosPublisherNode::callbackTimer, this);
}

void HelloRosPublisherNode::deactivating(void)
{
    ROS_INFO_STREAM(ros::this_node::getName() + __PRETTY_FUNCTION__);
    _timer.stop();
}

void HelloRosPublisherNode::shuttingDown(void)
{
    ROS_INFO_STREAM(ros::this_node::getName() + __PRETTY_FUNCTION__);
    ros::shutdown();
}

void HelloRosPublisherNode::callbackTimer(const ros::TimerEvent& event)
{
    ROS_INFO_STREAM(ros::this_node::getName() + __PRETTY_FUNCTION__);
    std_msgs::String msg;

    msg.data = "Hello ROS! My name is " + ros::this_node::getName();

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
