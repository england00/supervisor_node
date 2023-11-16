#include <chrono>
#include <ctime>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <random>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "simple_node/node.hpp"
#include "yasmin/state.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"

#define I "IDLE"
#define M "MANUAL"
#define A "ACTIVE"
#define ET "EMERGENCY TAKEOVER"
#define ES "EMERGENCY STOP"
#define END "END"
#define GENERAL_SLEEP 100
#define MANUAL_PUBLISHER_SLEEP 1000
#define FAULT_ALERT_SLEEP 1000
#define END_SLEEP 500
#define CURRENT_STATE_TOPIC "supervisor_node/current_state"
#define STATE_SELECTION_TOPIC "supervisor_node/state_selection"
#define MANUAL_COMMAND_TOPIC "supervisor_node/manual_command"
#define PRIMARY_DRIVING_STACK_TOPIC "supervisor_node/primary_driving_stack"
#define COMMON_FAULT_TOPIC "supervisor_node/common_fault"
#define SERIOUS_FAULT_TOPIC "supervisor_node/serious_fault"

using namespace std;

/***************************************************** Methods ********************************************************/
atomic_bool stop_thread;

void publisher(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub, const std::string &msg) {
    auto message = std_msgs::msg::String();
    message.data = msg;
    pub->publish(message);
}

void polling_publisher(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub, const std::string &msg) {
    auto message = std_msgs::msg::String();
    stop_thread = false;
    while (!stop_thread) {
        message.data = msg;
        pub->publish(message);
        this_thread::sleep_for(chrono::milliseconds(GENERAL_SLEEP));  // timer
    }
}

string timestamp(string command) {
    // date and time
    char time_buffer[50];  // buffer for date and current time
    auto now = chrono::system_clock::now();  // current date
    auto time = chrono::system_clock::to_time_t(now);  // current time
    strftime(time_buffer, sizeof(time_buffer), "%Y-%m-%d %H:%M:%S", localtime(&time));

    // milliseconds
    char millis_buffer[10]; // buffer for milliseconds
    auto millis = chrono::duration_cast<chrono::milliseconds>(now.time_since_epoch()) % 1000;
    snprintf(millis_buffer, sizeof(millis_buffer), ".%03d", static_cast<int>(millis.count()));

    // concatenate the two buffers
    strcat(time_buffer, millis_buffer);

    return "\nSent message on " + std::string(time_buffer) + ":\n --> " + command;
}

int generateRandomInt(int min, int max) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(min, max);
    return dis(gen);
}


/**************************************************** Idle State ******************************************************/
class IdleState : public yasmin::State {
private:
    // parameters
    string current_state_ = I;

public:
    // constructor
    IdleState() : yasmin::State({"I>M", "I>End"}) {};

    /// methods
    string execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
        blackboard->set<string>("previous_state", IdleState::to_string());  // memorizing last state

        // state cycle
        do {
            // managing what printing on stdout
            system("clear");
            cout << "PUB/SUB SIMULATOR NODE:\n\n"
                    "SUPERVISOR NODE --> ON\n"
                    "Current state: " <<
                    this->current_state_ << "\n\n\"The node is on and awaits signals from the outside." << endl;
            this_thread::sleep_for(chrono::milliseconds(GENERAL_SLEEP));  // timer

            // managing transitions
            if (this->current_state_ == M) {  // checking if SUPERVISOR NODE has switched to MANUAL state
                return "I>M";
            } else if (this->current_state_ == END) {  // checking if SUPERVISOR NODE has switched to END state
                return "I>End";
            }
        } while(true);
    }

    void set_current_state(string str) {
        this->current_state_ = str;
    }

    string to_string() {
        return I;
    }
};


/*************************************************** Manual State *****************************************************/
class ManualState : public yasmin::State {
private:
    // parameters
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr manual_commands_publisher_{};
    string current_state_ = M;

public:
    // constructor
    ManualState(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub) : yasmin::State({"M>I", "M>A", "M>ES"}),
                                                                            manual_commands_publisher_(pub) {};

    /// methods
    string execute(shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
        blackboard->set<string>("previous_state", ManualState::to_string());  // memorizing last state

        // state cycle
        do {
            // managing what printing on stdout
            system("clear");
            cout << "PUB/SUB SIMULATOR NODE:\n\n"
                    "SUPERVISOR NODE --> ON\n"
                    "Current state: " <<
                    this->current_state_ << "\n\nPublishing MANUAL COMMANDS sent." << endl;
            cout << timestamp(choose_manual_commands()) << endl;
            this_thread::sleep_for(chrono::milliseconds(MANUAL_PUBLISHER_SLEEP));  // timer

            // managing transitions
            if (this->current_state_ == I) {  // checking if SUPERVISOR NODE has switched to IDLE state
                return "M>I";
            } else if (this->current_state_ == A) {  // checking if SUPERVISOR NODE has switched to ACTIVE state
                return "M>A";
            }
        } while(true);
    }

    string choose_manual_commands() {
        int choise = generateRandomInt(1, 5);
        string command;
        if (choise == 1) {
            command = "GO STRAIGHT";
        } else if (choise == 2) {
            command = "TURN RIGHT";
        } else if (choise == 3) {
            command = "TURN LEFT";
        } else if (choise == 4) {
            command = "GO BACK RIGHT";
        } else if (choise == 5) {
            command = "STOP";
        }
        publisher(this->manual_commands_publisher_, command);
        return command;
    }

    void set_current_state(string str) {
        this->current_state_ = str;
    }

    string to_string() {
        return M;
    }
};


/*************************************************** Active State *****************************************************/
class ActiveState : public yasmin::State {
private:
    // parameters
    string current_state_ = A;

public:
    // constructor
    ActiveState() : yasmin::State({"A>M", "A>ET", "A>ES"}) {};

    /// methods
    string execute(shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
        blackboard->set<string>("previous_state", ActiveState::to_string());  // memorizing last state

        // state cycle
        do {
            // managing what printing on stdout
            system("clear");
            cout << "PUB/SUB SIMULATOR NODE:\n\n"
                    "SUPERVISOR NODE --> ON\n"
                    "Current state: " <<
                 this->current_state_ << "\n\nPublishing PRIMARY DRIVING STACK stack." << endl;
            this_thread::sleep_for(chrono::milliseconds(GENERAL_SLEEP));  // timer

            // managing transitions
            if (this->current_state_ == M) {  // checking if SUPERVISOR NODE has switched to MANUAL state
                return "A>M";
            }
        } while(true);
    }

    void set_current_state(string str) {
        this->current_state_ = str;
    }

    string to_string() {
        return A;
    }
};


/********************************************** Pub/Sub Simulator Node ************************************************/
class PubSubSimulatorNode : public simple_node::Node {
private:
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard_ = std::make_shared<yasmin::blackboard::Blackboard>();

    // publishers and subscribers
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr current_state_subscription_ = this->create_subscription<std_msgs::msg::String>(
            CURRENT_STATE_TOPIC,
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
            std::bind(&PubSubSimulatorNode::current_state_subscription, this, placeholders::_1)
    );
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publish_manual_command_ = this->create_publisher<std_msgs::msg::String>(
            MANUAL_COMMAND_TOPIC,
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable()
    );
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publish_primary_driving_stack_command_ = this->create_publisher<std_msgs::msg::String>(
            PRIMARY_DRIVING_STACK_TOPIC,
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable()
    );

    // states
    std::shared_ptr<IdleState> idleState_ = std::make_shared<IdleState>();
    std::shared_ptr<ManualState> manualState_ = std::make_shared<ManualState>(this->publish_manual_command_);
    std::shared_ptr<ActiveState> activeState_ = std::make_shared<ActiveState>();

    // threads for simulation
    thread primary_driving_stack_thread_;

public:
    // constructor
    PubSubSimulatorNode() : simple_node::Node("pub_sub_simulator_node") {

        // create a finite state machine
        auto fsm = std::make_shared<yasmin::StateMachine>(yasmin::StateMachine({END}));

        // add states
        fsm->add_state(this->idleState_->to_string(), this->idleState_, {
                {"I>M", this->manualState_->to_string()},
                {"I>End", END}
        });
        fsm->add_state(this->manualState_->to_string(), this->manualState_, {
                {"M>I", this->idleState_->to_string()},
                {"M>A", this->activeState_->to_string()}
        });
        fsm->add_state(this->activeState_->to_string(), this->activeState_, {
                {"A>M", this->manualState_->to_string()}
        });

        // executing fsm and parallel threads
        this->primary_driving_stack_thread_ = thread([this]() {  // opening new simulation thread
            polling_publisher(this->publish_primary_driving_stack_command_, "PRIMARY DRIVING STACK");
        });
        fsm->execute(this->blackboard_);

        // stopping parallel threads
        stop_thread = true;
        this->primary_driving_stack_thread_.join();

        // managing what printing on stdout
        system("clear");
        cout << "PUB/SUB SIMULATOR NODE:\n\nSUPERVISOR NODE --> OFF" << endl;
        this_thread::sleep_for(chrono::milliseconds(END_SLEEP));  // timer
        exit(EXIT_SUCCESS);
    }

    // passing SUPERVISOR NODE CURRENT STATE
    void current_state_subscription(const std_msgs::msg::String::SharedPtr msg) {
        this->idleState_->set_current_state(msg->data);
        this->manualState_->set_current_state(msg->data);
        this->activeState_->set_current_state(msg->data);
    }
};


/******************************************************* Main *********************************************************/
int main(int argc, char *argv[]) {
    // managing what printing on stdout
    cout << "PUB/SUB SIMULATOR NODE:\n\nSUPERVISOR NODE --> OFF\n" << endl;

    // C++ idiomatic interface which provides all the ROS client functionality like creating nodes, publisher, and subscribers
    rclcpp::init(argc, argv);  // activation of rclcpp API
    rclcpp::spin(std::make_shared<PubSubSimulatorNode>());  // node creation and spinning
    rclcpp::shutdown();  // shutdown of rclcpp API
    return 0;
}