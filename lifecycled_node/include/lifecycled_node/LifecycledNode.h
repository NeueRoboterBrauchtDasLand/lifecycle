#ifndef ___ROS_LIFECYCLE_H___
#define ___ROS_LIFECYCLE_H___

#include <ros/ros.h>

#include "lifecycle_msgs/CppNodeStatus.h"
#include "lifecycle_msgs/Lifecycle.h"

namespace lifecycled_node {

using lifecycle_msgs::cpp::NodeStatus;

class LifecycledNode
{
public:
    /**
     * \brief Default constructor.
     *
     * After construction the node state is CREATED. That means a status of this node isn't be published.
     * It is necessary to initialize this node by calling initializeLifecycle().
     */
    LifecycledNode(void);
    /**
     * \brief Constructs and initialize this node.
     *
     * This constructor calls internally initializeLifecycle(). The node state is UNCONFIGURED after construction.
     *
     * \param privNh Private node handle of this node.
     * \param nh Public node handle of this node.
     */
    LifecycledNode(ros::NodeHandle& privNh, ros::NodeHandle& nh);
    LifecycledNode(const LifecycledNode&) = delete;
    LifecycledNode(LifecycledNode&&)      = delete;
    virtual ~LifecycledNode(void);

    LifecycledNode& operator=(const LifecycledNode&) = delete;
    LifecycledNode& operator=(LifecycledNode&&)      = delete;

protected:
    void initializeLifecycle(ros::NodeHandle& privNh, ros::NodeHandle& nh);

    virtual void cleanup(void) = 0;
    virtual void configure(void) = 0;
    virtual void activating(void) = 0;
    virtual void deactivating(void) = 0;
    virtual void shuttingDown(void) = 0;

private:
    void onCleanup(void);
    void onConfigure(void);
    void onActivating(void);
    void onDeactivating(void);
    void onShutdown(void);

    void processLifecycle(const ros::TimerEvent& event);
    bool processServiceRequest(lifecycle_msgs::Lifecycle::Request& req, lifecycle_msgs::Lifecycle::Response& res);

    ros::Timer _timer;
    ros::ServiceServer _srvResponser;
    ros::Publisher _pubState;

    double _processingFreq = 1.0;
    NodeStatus::State _nodeState;
    std::string _nodeGroup;

    enum class DoExecute : std::uint8_t {
        NONE,
        CLEANUP,
        CONFIGURE,
        ACTIVATE,
        DEACTIVATE,
        SHUTDOWN
    };

    DoExecute _doExecute = DoExecute::NONE;
};

} // end namespace lifecycled_node

#endif
