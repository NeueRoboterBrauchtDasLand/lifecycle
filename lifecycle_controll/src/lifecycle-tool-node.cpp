#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <mutex>

#include <ros/ros.h>

#include <lifecycle_msgs/LifecycleControllerAction.h>
#include <lifecycle_msgs/NodeStatusArray.h>

class UserInterface
{
public:
    enum class Command : char {
        PRINT_NODES = 'p',
        CONFIGURE_NODES = 'c',
        UNCONFIGURED_NODES = 'u',
        ACTIVATE_NODES = 'a',
        DEACTIVATE_NODES = 'd',
        SHUTDOWN_NODES = 's',
        NONE
    };

    UserInterface(void)
        : _thread(&UserInterface::process, this)
    {

    }

    void process(void)
    {
        while (ros::ok())
        {
            if (_command == Command::NONE)
            {
                this->printMenu();

                char input;
                std::cin >> input;
                _command = static_cast<Command>(input);
            }
        }
    }

    void setNodes(const lifecycle_msgs::NodeStatusArray& msg)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _nodeNames.resize(msg.states.size());
        _lifecycles.resize(msg.states.size());

        for (unsigned int i = 0; i < msg.states.size(); ++i)
        {
            _nodeNames[i] = msg.states[i].node_name;
            _lifecycles[i] = std::to_string(msg.states[i].lifecycle);
        }
    }

    Command takeCommand(void)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        Command ret = _command;
        _command = Command::NONE;

        return ret;
    }

    std::string targetNode(void)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        return !_nodeNames.size() ? "none" : _nodeNames[_targetNodeIdx];
    }

private:
    void printMenu(void)
    {
        std::cout << "=== MENU ===" << std::endl;
        std::cout << "(p) Print all nodes." << std::endl;
        std::cout << "(c) Configure all nodes." << std::endl;
        std::cout << "(u) Unconfigure all nodes." << std::endl;
        std::cout << "(a) Activate all nodes." << std::endl;
        std::cout << "(d) Deactivate all node." << std::endl;
        std::cout << "(s) Shutdown all nodes." << std::endl;
        std::cout << std::endl;
        std::cout << "Select an action: ";
    }

    void printNodesAsList(void)
    {
        std::lock_guard<std::mutex> lock(_mutex);

        std::cout << "=== NODES ===" << std::endl;
        std::cout << std::endl;
        std::cout << "----------------------------------------------------------------------------------" << std::endl;
        std::cout << "| Name                                       | Lifecycle                          " << std::endl;
        std::cout << "----------------------------------------------------------------------------------" << std::endl;

        for (unsigned int i = 0; i < _nodeNames.size(); ++i)
            std::cout << "| " << _nodeNames[i] << " | " << _lifecycles[i] << std::endl;

        std::cout << "----------------------------------------------------------------------------------" << std::endl;
    }

    std::thread _thread;
    std::mutex _mutex;
    Command _command = Command::NONE;
    std::vector<std::string> _nodeNames;
    std::vector<std::string> _lifecycles;
    std::size_t _targetNodeIdx = 0;
};

void callbackStates(const lifecycle_msgs::NodeStatusArray& msg)
{
}

void callbackTimer(const ros::TimerEvent&)
{

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lifecycle_tool");
    ros::NodeHandle privNh("~"), nh;
    ros::Subscriber subStates(nh.subscribe("/lifecycle/controller/node_states", 1, callbackStates));
    ros::Timer timer(nh.createTimer(ros::Duration(0.1), callbackTimer));

    ros::spin();
}
