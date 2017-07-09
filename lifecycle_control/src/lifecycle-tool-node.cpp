#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>

#include <ros/ros.h>

#include <lifecycle_msgs/LifecycleControllerAction.h>
#include <lifecycle_msgs/NodeStatusArray.h>
#include <lifecycle_msgs/cpp/NodeStatus.h>

class UserInterface
{
public:
    enum class Command : char {
        PRINT_NODES = 'p',
        CONFIGURE = 'c',
        UNCONFIGURE = 'u',
        ACTIVATE = 'a',
        DEACTIVATE = 'd',
        SHUTDOWN = 's',
        SELECT_NODE = 'n',
        SELECT_GROUP = 'g',
        SELECT_ALL = 'a',
        CANCEL = 'c',
        EXIT = 'e',
        NONE = ' '
    };

    UserInterface(void)
        : _thread(&UserInterface::process, this)
    {

    }

    void process(void)
    {
        while (ros::ok())
        {
            this->selectTarget();
        }
    }

    void setNodes(const lifecycle_msgs::NodeStatusArray& msg)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _nodeNames.resize(msg.states.size());
        _lifecycles.resize(msg.states.size());
        _groups = msg.groups;

        for (unsigned int i = 0; i < msg.states.size(); ++i)
        {
            _nodeNames[i] = msg.states[i].node_name;
            _lifecycles[i] = lifecycle_msgs::cpp::NodeStatus::stateName(static_cast<lifecycle_msgs::cpp::NodeStatus::State>(msg.states[i].lifecycle));
        }
    }

    Command takeCommand(void)
    {
        Command ret = _command;
        _command = Command::NONE;

        return ret;
    }

    std::string targetNode(void)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        return _targetNode;
    }

private:
    void printActionMenu(void)
    {
        std::cout << "=== ACTION ===" << std::endl;
        std::cout << "(c) Configure" << std::endl;
        std::cout << "(u) Unconfigure" << std::endl;
        std::cout << "(a) Activate" << std::endl;
        std::cout << "(d) Deactivate" << std::endl;
        std::cout << "(s) Shutdown" << std::endl;
        std::cout << std::endl;
        std::cout << "(n) None" << std::endl;
    }

    void printSelectOptions(void)
    {
        std::cout << "=== SELECT TARGET ===" << std::endl;
        std::cout << "(n) Select a specific node." << std::endl;
        std::cout << "(g) Select a group of nodes." << std::endl;
        std::cout << "(a) Select all nodes." << std::endl;
        std::cout << "(p) Print all node." << std::endl;
        std::cout << std::endl;
        std::cout << "(e) Exit" << std::endl;
        std::cout << std::endl;
    }

    void printNodesAsList(void)
    {
        std::lock_guard<std::mutex> lock(_mutex);

        std::cout << "+-----+---------------------------------------------------------+-----------------+" << std::endl;
        std::cout << "| Nr. | Name                                                    | Lifecycle       |" << std::endl;
        std::cout << "+-----+---------------------------------------------------------+-----------------|" << std::endl;

        for (unsigned int i = 0; i < _nodeNames.size(); ++i)
        {
            std::cout << "| " << std::setw(3) << i + 1 << " | " << std::setw(55) << _nodeNames[i] << " | ";
            std::cout << std::setw(15) << _lifecycles[i] << " |" << std::endl;

            if (i && !(i % 5))
                std::cout << "+-----+---------------------------------------------------------+-----------------+" << std::endl;
        }

        std::cout << "+-----+---------------------------------------------------------+-----------------+" << std::endl;
    }

    void printGroupsAsList(void)
    {
        std::lock_guard<std::mutex> lock(_mutex);

        std::cout << "+-----+---------------------------------------------------------+" << std::endl;
        std::cout << "| Nr. | Group                                                   |" << std::endl;
        std::cout << "+-----+---------------------------------------------------------+" << std::endl;

        for (unsigned int i = 0; i < _groups.size(); ++i)
        {
            std::cout << "| " << std::setw(3) << i + 1 << " | " << std::setw(55) << _groups[i] << " |" << std::endl;

            if (i && !(i % 5))
                std::cout << "+-----+---------------------------------------------------------+" << std::endl;
        }

        std::cout << "+-----+---------------------------------------------------------+" << std::endl;
    }

    void selectTarget(void)
    {
    	Command command;

        do
        {
            this->printSelectOptions();
            std::cout << "Select a target: ";

            char input;
            std::cin.clear();
            std::cin >> input;
            std::cout << std::endl << std::endl;
            command = static_cast<Command>(input);

            switch (command)
            {
            case Command::SELECT_NODE:

                if (!_nodeNames.size())
                {
                    std::cout << "No nodes in the list! Can't select one, seems no lifecycled nodes exists." << std::endl;
                    break;
                }

                if (!this->selectNode())
                    break;

                this->selectCommand();
                break;

            case Command::SELECT_GROUP:

                if (!this->selectGroup())
                    break;

                this->selectCommand();
                break;

            case Command::SELECT_ALL:
                _targetNode = "broadcast";
                this->selectCommand();
                break;

            case Command::PRINT_NODES:
                this->printNodesAsList();
                break;

            case Command::EXIT:
                ros::shutdown();
                std::cout << "Will exit..." << std::endl;
                return;

            default:
                std::cout << "Invalid input! Try again..." << std::endl;
                break;
            }
        }
        while (command == Command::NONE);

        std::cout << std::endl << std::endl;
    }

    bool selectNode(void)
    {
        int indexNode = -1;

        do
        {
            std::cout << "Please select one node from the following list." << std::endl;
            this->printNodesAsList();
            std::cout << "Select node by number (1 .. " << _nodeNames.size() << " or 0 to cancel): ";

            std::cin >> indexNode;
        }
        while (indexNode < 0 || indexNode > _nodeNames.size());

        if (!indexNode)
            return false;

        _targetNode = _nodeNames[indexNode - 1];
        std::cout << std::endl;
        return true;
    }

    bool selectGroup(void)
    {
        int indexGroup = -1;

        do
        {
            std::cout << "Please select one group of the following list." << std::endl;
            this->printGroupsAsList();
            std::cout << "Select group by number (1 .. " << _groups.size() << " or 0 to cancel): ";

            std::cin >> indexGroup;
        }
        while (indexGroup < 0 || indexGroup > _groups.size());

        if (!indexGroup)
            return false;

        _targetNode = _groups[indexGroup - 1];
        std::cout << std::endl;
        return true;
    }

    bool selectCommand(void)
    {
        char input;
        Command command;

        do
        {
            this->printActionMenu();
            std::cout << "Select an action: ";
            std::cin >> input;
            command = static_cast<Command>(input);

            if (command == static_cast<Command>('n')) // Hack here! CANCEL doesn't work here, because 'c' is already in use.
                return false;
        }
        while (command != Command::CONFIGURE && command != Command::ACTIVATE && command != Command::DEACTIVATE &&
               command != Command::UNCONFIGURE && command != Command::SHUTDOWN);

        _command = command;
        return true;
    }

    std::thread _thread;
    std::mutex _mutex;
    std::atomic<Command> _command{Command::NONE};
    std::vector<std::string> _nodeNames;
    std::vector<std::string> _lifecycles;
    std::vector<std::string> _groups;
    std::string _targetNode;
};

UserInterface _ui;
ros::ServiceClient _srvController;

void callbackStates(const lifecycle_msgs::NodeStatusArray& msg)
{
    _ui.setNodes(msg);
}

void callbackTimer(const ros::TimerEvent&)
{
    const UserInterface::Command command = _ui.takeCommand();
    lifecycle_msgs::LifecycleControllerAction msg;

    switch (command)
    {
    case UserInterface::Command::CONFIGURE:
        msg.request.action = lifecycle_msgs::LifecycleControllerAction::Request::ACTION_CHANGE_LIFECYCLE;
        msg.request.target_node = _ui.targetNode();
        msg.request.target_lifecycle = static_cast<std::uint8_t>(lifecycle_msgs::cpp::NodeStatus::State::INACTIVE);

        if (!_srvController.call(msg)) ROS_ERROR_STREAM(ros::this_node::getName() + ": can't call service");
        break;

    case UserInterface::Command::UNCONFIGURE:
        msg.request.action = lifecycle_msgs::LifecycleControllerAction::Request::ACTION_CHANGE_LIFECYCLE;
        msg.request.target_node = _ui.targetNode();
        msg.request.target_lifecycle = static_cast<std::uint8_t>(lifecycle_msgs::cpp::NodeStatus::State::UNCONFIGURED);

        if (!_srvController.call(msg)) ROS_ERROR_STREAM(ros::this_node::getName() + ": can't call service");
        break;

    case UserInterface::Command::ACTIVATE:
        msg.request.action = lifecycle_msgs::LifecycleControllerAction::Request::ACTION_CHANGE_LIFECYCLE;
        msg.request.target_node = _ui.targetNode();
        msg.request.target_lifecycle = static_cast<std::uint8_t>(lifecycle_msgs::cpp::NodeStatus::State::ACTIVE);

        if (!_srvController.call(msg)) ROS_ERROR_STREAM(ros::this_node::getName() + ": can't call service");
        break;

    case UserInterface::Command::DEACTIVATE:
        msg.request.action = lifecycle_msgs::LifecycleControllerAction::Request::ACTION_CHANGE_LIFECYCLE;
        msg.request.target_node = _ui.targetNode();
        msg.request.target_lifecycle = static_cast<std::uint8_t>(lifecycle_msgs::cpp::NodeStatus::State::INACTIVE);

        if (!_srvController.call(msg)) ROS_ERROR_STREAM(ros::this_node::getName() + ": can't call service");
        break;

    case UserInterface::Command::SHUTDOWN:
        msg.request.action = lifecycle_msgs::LifecycleControllerAction::Request::ACTION_CHANGE_LIFECYCLE;
        msg.request.target_node = _ui.targetNode();
        msg.request.target_lifecycle = static_cast<std::uint8_t>(lifecycle_msgs::cpp::NodeStatus::State::FINALIZED);

        if (!_srvController.call(msg)) ROS_ERROR_STREAM(ros::this_node::getName() + ": can't call service");
        break;

    default:
        break;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lifecycle_tool");
    ros::NodeHandle privNh("~"), nh;
    ros::Subscriber subStates(nh.subscribe("/lifecycle/controller/node_states", 1, callbackStates));
    _srvController = nh.serviceClient<lifecycle_msgs::LifecycleControllerAction>("/lifecycle/service/controller");
    ros::Timer timer(nh.createTimer(ros::Duration(0.1), callbackTimer));

    ros::spin();
}
