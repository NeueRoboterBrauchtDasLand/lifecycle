#ifndef ___HELLO_ROS_PUBLISHER_H___
#define ___HELLO_ROS_PUBLISHER_H___

#include "lifecycled_node/LifecycledNode.h"

namespace lifecycled_node {

namespace test {

class HelloRosPublisherNode : public lifecycled_node::LifecycledNode
{
public:

    HelloRosPublisherNode(ros::NodeHandle& privNh, ros::NodeHandle& nh);
    virtual ~HelloRosPublisherNode(void);

    void exec(void);

protected:

    virtual void cleanup(void);
    virtual void configure(void);
    virtual void activating(void);
    virtual void deactivating(void);
    virtual void shuttingDown(void);

private:

    void callbackTimer(const ros::TimerEvent& event);

    ros::NodeHandle& _nh;
    double _publishFreq;
    ros::Publisher _pubHelloRos;
    ros::Timer _timer;
};

} // end namespace test

} // end namespace lifecycled_node

#endif
