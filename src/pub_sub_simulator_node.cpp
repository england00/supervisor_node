#include <chrono>
#include <ctime>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <mutex>
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
#define GENERAL_SLEEP 100  // milliseconds
#define MANUAL_COMMAND_PUBLISHER_SLEEP 1000  // milliseconds
#define PRIMARY_DRIVING_STACK_PUBLISHER_SLEEP 100  // milliseconds
#define SECONDARY_DRIVING_STACK_PUBLISHER_SLEEP 100  // milliseconds
#define END_SLEEP 500  // milliseconds
#define CURRENT_STATE_TOPIC "supervisor_node/current_state"
#define STATE_SELECTION_TOPIC "supervisor_node/state_selection"
#define MANUAL_COMMAND_TOPIC "supervisor_node/manual_command"
#define PRIMARY_DRIVING_STACK_TOPIC "supervisor_node/primary_driving_stack"
#define SECONDARY_DRIVING_STACK_TOPIC "supervisor_node/secondary_driving_stack"
#define COMMON_FAULT_TOPIC "supervisor_node/common_fault"
#define SERIOUS_FAULT_TOPIC "supervisor_node/serious_fault"

using namespace std;

/***************************************************** Methods ********************************************************/
int generateRandomInt(int min, int max) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(min, max);
    return dis(gen);
}

string choose_manual_commands() {
    int choise = generateRandomInt(1, 5);
    string command;
    if (choise == 1) {  command = "GO STRAIGHT";  }
    else if (choise == 2) {  command = "TURN RIGHT";  }
    else if (choise == 3) {  command = "TURN LEFT";  }
    else if (choise == 4) {  command = "GO BACK RIGHT";  }
    else if (choise == 5) {  command = "STOP";  }
    return command;
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

void end_execution() {
    system("clear");
    cout << "PUB/SUB SIMULATOR NODE:\n\nSUPERVISOR NODE --> OFF" << endl;
    this_thread::sleep_for(chrono::milliseconds(END_SLEEP));  // timer
    exit(EXIT_SUCCESS);
}

void publisher(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub, const std::string &msg) {
    auto message = std_msgs::msg::String();
    message.data = msg;
    pub->publish(message);
}

atomic_bool stop_thread;
void polling_publisher(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub, const std::string &state, int delay) {
    auto message = std_msgs::msg::String();
    string commands;
    stop_thread = false;
    while (!stop_thread) {
        // different published messages based on current state
        if (state == M) {  // MANUAL STATE
            message.data = choose_manual_commands();
            commands = "MANUAL COMMANDS";
        } else if (state == A) {
            message.data = "PRIMARY DRIVING STACK";
            commands = "PRIMARY DRIVING STACK";
        } else if (state == ET) {
            message.data = "SECONDARY DRIVING STACK";
            commands = "SECONDARY DRIVING STACK";
        }
        system("clear");
        cout << "PUB/SUB SIMULATOR NODE:\n\n"
                "SUPERVISOR NODE --> ON\n"
                "Current state: " <<
                 state << "\n\nPublishing " << commands << ":" << endl;
        pub->publish(message);  // publishing
        cout << timestamp(message.data) << endl;
        this_thread::sleep_for(chrono::milliseconds(delay));  // timer
    }
}


/**************************************************** Idle State ******************************************************/
class IdleState : public yasmin::State {
private:
    // parameters
    string selected_state_, current_state_;

public:
    /// constructor
    IdleState() : yasmin::State({"I>M", "I>End"}) {};

    /// execution
    string execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
        blackboard->set<string>("previous_state", IdleState::to_string());  // memorizing last state

        // state cycle
        do {
            // managing what printing on stdout
            if (!this->current_state_.empty()) {
                system("clear");
                cout << "PUB/SUB SIMULATOR NODE:\n\n"
                        "SUPERVISOR NODE --> ON\n"
                        "Current state: " <<
                     this->current_state_ << "\n\n\"The node is on and awaits signals from the outside." << endl;
            }
            this_thread::sleep_for(chrono::milliseconds(GENERAL_SLEEP));  // timer

            // managing transitions
            if (this->selected_state_ == M) {  // checking if SUPERVISOR NODE has switched to MANUAL state
                return "I>M";
            } else if (this->selected_state_ == END) {  // checking if SUPERVISOR NODE has switched to END state
                return "I>End";
            }
        } while(this->current_state_ != END);

        // emergency exit
        end_execution();
        return "#";
    }

    /// other methods
    void set_selected_state(string str) {  this->selected_state_ = str;  }
    void set_current_state(string str) {  this->current_state_ = str;  }
    string to_string() {  return I;  }
};


/*************************************************** Manual State *****************************************************/
class ManualState : public yasmin::State {
private:
    // parameters
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr manual_commands_publisher_{};
    string selected_state_, current_state_;
    thread manual_commands_thread_;

public:
    /// constructor
    ManualState(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub) : yasmin::State({"M>I", "M>A", "M>ES"}),
                                                                            manual_commands_publisher_(pub) {};
    /// execution
    string execute(shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
        blackboard->set<string>("previous_state", ManualState::to_string());  // memorizing last state

        // starting publisher thread
        this->manual_commands_thread_ = thread([this]() {  // opening new simulation thread
            polling_publisher(this->manual_commands_publisher_, ManualState::to_string(), MANUAL_COMMAND_PUBLISHER_SLEEP);
        });

        // state cycle
        do {
            // timer
            this_thread::sleep_for(chrono::milliseconds(GENERAL_SLEEP));

            // managing transitions
            if (this->selected_state_ == I) {  // checking if SUPERVISOR NODE has switched to IDLE state
                this->exit_publisher_thread();
                return "M>I";
            } else if (this->selected_state_ == A) {  // checking if SUPERVISOR NODE has switched to ACTIVE state
                this->exit_publisher_thread();
                return "M>A";
            }
        } while(this->current_state_ != END);

        // emergency exit
        end_execution();
        return "#";
    }

    /// other methods
    void exit_publisher_thread() {
        stop_thread = true;
        this->manual_commands_thread_.join();
    }
    void set_selected_state(string str) {  this->selected_state_ = str;  }
    void set_current_state(string str) {  this->current_state_ = str;  }
    string to_string() {  return M;  }
};


/*************************************************** Active State *****************************************************/
class ActiveState : public yasmin::State {
private:
    // parameters
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr primary_driving_stack_publisher_{};
    string selected_state_, current_state_;
    thread primary_driving_stack_thread_;

public:
    /// constructor
    ActiveState(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub) : yasmin::State({"A>M", "A>ET", "A>ES"}),
                                                                            primary_driving_stack_publisher_(pub) {};
    /// execution
    string execute(shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
        blackboard->set<string>("previous_state", ActiveState::to_string());  // memorizing last state

        // starting publisher thread
        this->primary_driving_stack_thread_ = thread([this]() {  // opening new simulation thread
            polling_publisher(this->primary_driving_stack_publisher_, ActiveState::to_string(), PRIMARY_DRIVING_STACK_PUBLISHER_SLEEP);
        });

        // state cycle
        do {
            // timer
            this_thread::sleep_for(chrono::milliseconds(GENERAL_SLEEP));

            // managing transitions
            if (this->selected_state_ == M) {  // checking if SUPERVISOR NODE has switched to MANUAL state
                this->exit_publisher_thread();
                return "A>M";
            } else if (this->current_state_ == ET) {  // checking if SUPERVISOR NODE has switched to EMERGENCY TAKEOVER state
                this->exit_publisher_thread();
                return "A>ET";
            }
        } while(this->current_state_ != END);

        // emergency exit
        end_execution();
        return "#";
    }

    /// other methods
    void exit_publisher_thread() {
        stop_thread = true;
        this->primary_driving_stack_thread_.join();
    }
    void set_selected_state(string str) {  this->selected_state_ = str;  }
    void set_current_state(string str) {  this->current_state_ = str;  }
    string to_string() {  return A;  }
};

/********************************************* Emergency Takeover State ***********************************************/
class EmergencyTakeoverState : public yasmin::State {
private:
    // parameters
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr secondary_driving_stack_publisher_{};
    string selected_state_, current_state_;
    thread secondary_driving_stack_thread_;

public:
    /// constructor
    EmergencyTakeoverState(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub) : yasmin::State({"ET>M", "ET>A", "ET>ES"}),
                                                                                       secondary_driving_stack_publisher_(pub) {};
    /// execution
    string execute(shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
        blackboard->set<string>("previous_state", EmergencyTakeoverState::to_string());  // memorizing last state

        // starting publisher thread
        this->secondary_driving_stack_thread_ = thread([this]() {  // opening new simulation thread
            polling_publisher(this->secondary_driving_stack_publisher_, EmergencyTakeoverState::to_string(), SECONDARY_DRIVING_STACK_PUBLISHER_SLEEP);
        });

        // state cycle
        do {
            // timer
            this_thread::sleep_for(chrono::milliseconds(GENERAL_SLEEP));

            // managing transitions
            if (this->current_state_ == A) {  // checking if SUPERVISOR NODE has switched to ACTIVE state
                this->exit_publisher_thread();
                return "ET>A";
            }
        } while(this->current_state_ != END);

        // emergency exit
        end_execution();
        return "#";
    }

    /// other methods
    void exit_publisher_thread() {
        stop_thread = true;
        this->secondary_driving_stack_thread_.join();
    }
    void set_selected_state(string str) {  this->selected_state_ = str;  }
    void set_current_state(string str) {  this->current_state_ = str;  }
    string to_string() {  return ET;  }
};


/********************************************** Pub/Sub Simulator Node ************************************************/
class PubSubSimulatorNode : public simple_node::Node {
private:
    // blackboard
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard_ = std::make_shared<yasmin::blackboard::Blackboard>();

    // publishers and subscribers
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr selected_state_sub_ = nullptr;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr current_state_subscription_ = nullptr;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publish_manual_command_ = nullptr;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publish_primary_driving_stack_command_ = nullptr;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publish_secondary_driving_stack_command_ = nullptr;

    // states
    std::shared_ptr<IdleState> idleState_ = nullptr;
    std::shared_ptr<ManualState> manualState_ = nullptr;
    std::shared_ptr<ActiveState> activeState_ = nullptr;
    std::shared_ptr<EmergencyTakeoverState> emergencyTakeoverState_ = nullptr;

public:
    /// constructor
    PubSubSimulatorNode() : simple_node::Node("pub_sub_simulator_node") {

        // quality of service
        rclcpp::QoS qos_profile(rclcpp::KeepLast(1));  // queue dimension
        qos_profile.reliable();  // type of communication

        // publishers and subscribers
        /// SELECTED STATE SUBSCRIPTION
        this->selected_state_sub_ = this->create_subscription<std_msgs::msg::String>(STATE_SELECTION_TOPIC,
            qos_profile,
            std::bind(&PubSubSimulatorNode::selected_state_subscription, this, placeholders::_1)
        );
        /// CURRENT STATE SUBSCRIPTION
        qos_profile.transient_local();
        this->current_state_subscription_ = this->create_subscription<std_msgs::msg::String>(CURRENT_STATE_TOPIC,
            qos_profile,
            std::bind(&PubSubSimulatorNode::current_state_subscription, this, placeholders::_1)
        );
        /// MANUAL COMMANDS PUBLISHER
        this->publish_manual_command_ = this->create_publisher<std_msgs::msg::String>(MANUAL_COMMAND_TOPIC,
            qos_profile
        );
        /// PRIMARY DRIVING STACK PUBLISHER
        qos_profile.deadline(chrono::milliseconds(120));
        //qos_profile.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC).liveliness_lease_duration(chrono::milliseconds(2000));
        this->publish_primary_driving_stack_command_ = this->create_publisher<std_msgs::msg::String>(
                PRIMARY_DRIVING_STACK_TOPIC,
                qos_profile
        );
        /// SECONDARY DRIVING STACK PUBLISHER
        qos_profile.deadline(chrono::milliseconds(120));
        this->publish_secondary_driving_stack_command_ = this->create_publisher<std_msgs::msg::String>(
                SECONDARY_DRIVING_STACK_TOPIC,
                qos_profile
        );

        // create a finite state machine
        this->idleState_ = std::make_shared<IdleState>();
        this->manualState_ = std::make_shared<ManualState>(this->publish_manual_command_);
        this->activeState_ = std::make_shared<ActiveState>(this->publish_primary_driving_stack_command_);
        this->emergencyTakeoverState_ = std::make_shared<EmergencyTakeoverState>(this->publish_secondary_driving_stack_command_);
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
                {"A>M", this->manualState_->to_string()},
                {"A>ET", this->emergencyTakeoverState_->to_string()}
        });
        fsm->add_state(this->emergencyTakeoverState_->to_string(), this->emergencyTakeoverState_, {
                {"ET>M", this->manualState_->to_string()},
                {"ET>A", this->activeState_->to_string()}
        });

        // executing fsm
        fsm->execute(this->blackboard_);

        // exit
        end_execution();
    }

    /// other methods
    // passing SELECTED STATE
    void selected_state_subscription(const std_msgs::msg::String::SharedPtr msg) {
        this->idleState_->set_selected_state(msg->data);
        this->manualState_->set_selected_state(msg->data);
        this->activeState_->set_selected_state(msg->data);
        this->emergencyTakeoverState_->set_selected_state(msg->data);
    }
    // passing SUPERVISOR NODE CURRENT STATE
    void current_state_subscription(const std_msgs::msg::String::SharedPtr msg) {
        this->idleState_->set_current_state(msg->data);
        this->manualState_->set_current_state(msg->data);
        this->activeState_->set_current_state(msg->data);
        this->emergencyTakeoverState_->set_current_state(msg->data);
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