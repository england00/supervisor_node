#include <chrono>
#include <ctime>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

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

#define SLEEP 100
#define FAULT_ALERT_SLEEP 1000
#define CURRENT_STATE_TOPIC "supervisor_node/current_state"
#define STATE_SELECTION_TOPIC "supervisor_node/state_selection"
#define MANUAL_COMMAND_TOPIC "supervisor_node/manual_command"
#define PRIMARY_DRIVING_STACK_TOPIC "supervisor_node/primary_driving_stack"
#define COMMON_FAULT_TOPIC "supervisor_node/common_fault"
#define SERIOUS_FAULT_TOPIC "supervisor_node/serious_fault"

using namespace std;

/***************************************************** Methods ********************************************************/
void publishing(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub, const std::string &msg) {
    auto message = std_msgs::msg::String();
    message.data = msg;
    pub->publish(message);
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

    return "\n" + std::string(time_buffer) + " --> " + command;
}


/**************************************************** Idle State ******************************************************/
class IdleState : public yasmin::State {
private:
    // parameters
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr idle_state_pub_{};
    string next_state_;

public:
    // constructor
    IdleState(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub) : yasmin::State({"I>M", "I>End"}),
                                                                          idle_state_pub_(pub) {};
    /// methods
    string execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
        string stdout_message = "Executing state " + IdleState::to_string() + "\n\n"
                                "The node is on and awaits signals from the outside.";
        blackboard->set<string>("previous_state", IdleState::to_string());  // memorizing last state
        publishing(idle_state_pub_, IdleState::to_string());

        // state cycle
        do {
            system("clear");
            cout << "SUPERVISION NODE:\n" << endl;

            // managing what printing on stdout
            cout << stdout_message << endl;
            this_thread::sleep_for(chrono::milliseconds(SLEEP));  // timer

            // managing transitions
            if (this->next_state_ == M) {  // checking if MANUAL state has been selected
                return "I>M";
            } else if (this->next_state_ == END) {  // checking if END state has been selected
                return "I>End";
            }
        } while(true);
    }

    void set_next_state(string str) {  this->next_state_ = str;  }
    string to_string() {  return I;  }
};


/*************************************************** Manual State *****************************************************/
class ManualState : public yasmin::State {
private:
    // parameters
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr manual_state_pub_{};
    string next_state_, manual_command_;

public:
    // constructor
    ManualState(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub) : yasmin::State({"M>I", "M>A", "M>ES"}),
                                                                            manual_state_pub_(pub) {};
    /// methods
    string execute(shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
        string stdout_message = "Executing state " + ManualState::to_string() + "\n\n"
                                "The vehicle is in manual driving mode, therefore:\n"
                                "- no fault checks are performed;\n"
                                "- all control commands provided by the primary and secondary stack are ignored.";
        string stdout_buffer;
        blackboard->set<string>("previous_state", ManualState::to_string());  // memorizing last state
        publishing(manual_state_pub_, ManualState::to_string());

        // state cycle
        do {
            system("clear");
            cout << "SUPERVISION NODE:\n" << endl;

            // managing what printing on stdout
            cout << stdout_message << endl;  // default output
            if (!this->manual_command_.empty()) {  // adding the last command received
                stdout_buffer = timestamp(this->manual_command_);
            }
            cout << stdout_buffer << endl;
            this->manual_command_ = "";  // cleaning latest command buffer
            this_thread::sleep_for(chrono::milliseconds(SLEEP));  // timer

            // managing transitions
            if (this->next_state_ == I) {  // checking if IDLE state has been selected
                return "M>I";
            } else if (this->next_state_ == A) {  // checking if ACTIVE state has been selected
                return "M>A";
            }
        } while(true);
    }

    void set_next_state(string str) {  this->next_state_ = str;  }
    void set_manual_command(string str) {  this->manual_command_ = str;  }
    string to_string() {  return M;  }
};


/*************************************************** Active State *****************************************************/
class ActiveState : public yasmin::State {
private:
    // parameters
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_state_pub_{};
    string next_state_, primary_driving_stack_command_, common_fault_;

public:
    // constructor
    ActiveState(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub) : yasmin::State({"A>M", "A>ET", "A>ES"}),
                                                                            active_state_pub_(pub) {};
    /// methods
    string execute(shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
        string stdout_message = "Executing state " + ActiveState::to_string() + "\n\n"
                                "The vehicle is in autonomous driving mode, therefore:\n"
                                "- fault checks are performed;\n"
                                "- control is entrusted to the primary driving stack.";
        string stdout_buffer;
        this->common_fault_.clear();
        blackboard->set<string>("previous_state", ActiveState::to_string());  // memorizing last state
        publishing(active_state_pub_, ActiveState::to_string());

        // state cycle
        do {
            system("clear");
            cout << "SUPERVISION NODE:\n" << endl;

            // managing what printing on stdout
            cout << stdout_message << endl;  // default output
            if (!this->primary_driving_stack_command_.empty()) {  // adding the last command received
                stdout_buffer = timestamp(this->primary_driving_stack_command_);
            }
            cout << stdout_buffer << endl;
            this->primary_driving_stack_command_ = "";  // cleaning latest command buffer
            this_thread::sleep_for(chrono::milliseconds(SLEEP));  // timer

            // managing common fault
            if (!this->common_fault_.empty()) {  // adding the last command received
                cout << timestamp(this->primary_driving_stack_command_) << endl;
                this_thread::sleep_for(chrono::milliseconds(FAULT_ALERT_SLEEP));  // timer
                return "A>ET";
            }

            // managing transitions
            if (this->next_state_ == M) {  // checking if MANUAL state has been selected
                return "A>M";
            }
        } while(true);
    }

    void set_next_state(string str) {  this->next_state_ = str;  }
    void set_primary_driving_stack_command(string str) {  this->primary_driving_stack_command_ = str;  }
    void set_common_fault(string str) {  this->common_fault_ = str;  }
    string to_string() {  return A;  }
};


/********************************************* Emergency Takeover State ***********************************************/
class EmergencyTakeoverState : public yasmin::State {
private:
    // parameters
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr emergency_takeover_state_pub_{};
    string next_state_, common_fault_;

public:
    // constructor
    EmergencyTakeoverState(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub) : yasmin::State({"ET>M", "ET>A", "ET>ES"}),
                                                                                       emergency_takeover_state_pub_(pub) {};
    /// methods
    string execute(shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
        string stdout_message = "Executing state " + EmergencyTakeoverState::to_string() + "\n\n"
                                "The vehicle is in a state of risk:\n"
                                "- control is entrusted to the secondary driving stack.";
        blackboard->set<string>("previous_state", EmergencyTakeoverState::to_string());  // memorizing last state
        publishing(emergency_takeover_state_pub_, EmergencyTakeoverState::to_string());

        // state cycle
        do {
            system("clear");
            cout << "SUPERVISION NODE:\n" << endl;

            // managing what printing on stdout
            cout << stdout_message << endl;
            this_thread::sleep_for(chrono::milliseconds(SLEEP));  // timer
        } while(true);
    }

    void set_next_state(string str) {  this->next_state_ = str;  }
    string to_string() {  return ET;  }
};


/*********************************************** Emergency Stop State *************************************************/
class EmergencyStopState : public yasmin::State {
private:
    // parameters
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr emergency_stop_state_pub_{};
    string next_state_;

public:
    // constructor
    EmergencyStopState(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub) : yasmin::State({"ES>M", "ES>ET"}),
                                                                                   emergency_stop_state_pub_(pub) {};
    /// methods
    string execute(shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
        string stdout_message = "Executing state " + EmergencyStopState::to_string() + "\n\n"
                                "The vehicle is unable to move autonomously:\n"
                                "- driving commands are ignored and the vehicle is brought to a stop.";
        blackboard->set<string>("previous_state", EmergencyStopState::to_string());  // memorizing last state
        publishing(emergency_stop_state_pub_, EmergencyStopState::to_string());

        // state cycle
        do {
            system("clear");
            cout << "SUPERVISION NODE:\n" << endl;

            // managing what printing on stdout
            cout << stdout_message << endl;
            this_thread::sleep_for(chrono::milliseconds(SLEEP));  // timer
        } while(true);
    }

    void set_next_state(string str) {  this->next_state_ = str;  }
    string to_string() {  return ES;  }
};


/************************************************** Supervisor Node ***************************************************/
class SupervisorNode : public simple_node::Node {
private:
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard_ = std::make_shared<yasmin::blackboard::Blackboard>();

    // publishers and subscribers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr current_state_pub_ = this->create_publisher<std_msgs::msg::String>(
        CURRENT_STATE_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local()
    );
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr selected_state_sub_ = this->create_subscription<std_msgs::msg::String>(
        STATE_SELECTION_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
        std::bind(&SupervisorNode::selected_state_subscription, this, placeholders::_1)
    );
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr manual_command_sub_ = this->create_subscription<std_msgs::msg::String>(
        MANUAL_COMMAND_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
        std::bind(&SupervisorNode::manual_command_subscription, this, placeholders::_1)
    );
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr primary_driving_stack_sub_ = this->create_subscription<std_msgs::msg::String>(
        PRIMARY_DRIVING_STACK_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
        std::bind(&SupervisorNode::primary_driving_stack_subscription, this, placeholders::_1)
    );
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr common_fault_sub_ = this->create_subscription<std_msgs::msg::String>(
        COMMON_FAULT_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
        std::bind(&SupervisorNode::common_fault_subscription, this, placeholders::_1)
    );
    std::unique_ptr<yasmin_viewer::YasminViewerPub> yasmin_pub_{};

    // states
    std::shared_ptr<IdleState> idleState_ = std::make_shared<IdleState>(this->current_state_pub_);
    std::shared_ptr<ManualState> manualState_ = std::make_shared<ManualState>(this->current_state_pub_);
    std::shared_ptr<ActiveState> activeState_ = std::make_shared<ActiveState>(this->current_state_pub_);
    std::shared_ptr<EmergencyTakeoverState> emergencyTakeoverState_ = std::make_shared<EmergencyTakeoverState>(this->current_state_pub_);
    std::shared_ptr<EmergencyStopState> emergencyStopState_ = std::make_shared<EmergencyStopState>(this->current_state_pub_);

public:
    // constructor
    SupervisorNode() : simple_node::Node("supervisor_node") {

        // create a finite state machine
        auto fsm = std::make_shared<yasmin::StateMachine>(yasmin::StateMachine({END}));

        // publishing and subscribing for states information
        this->yasmin_pub_ = std::make_unique<yasmin_viewer::
        YasminViewerPub>(yasmin_viewer::YasminViewerPub(this, "SUPERVISOR_NODE", fsm));

        // add states
        fsm->add_state(this->idleState_->to_string(), this->idleState_, {
            {"I>M", this->manualState_->to_string()},
            {"I>End", END}
        });
        fsm->add_state(this->manualState_->to_string(), this->manualState_, {
            {"M>I", this->idleState_->to_string()},
            {"M>A", this->activeState_->to_string()},
            {"M>ES", this->emergencyStopState_->to_string()}
        });
        fsm->add_state(this->activeState_->to_string(), this->activeState_, {
            {"A>M", this->manualState_->to_string()},
            {"A>ET", this->emergencyTakeoverState_->to_string()},
            {"A>ES", this->emergencyStopState_->to_string()}
        });
        fsm->add_state(this->emergencyTakeoverState_->to_string(), this->emergencyTakeoverState_, {
            {"ET>M", this->manualState_->to_string()},
            {"ET>A", this->activeState_->to_string()},
            {"ET>ES", this->emergencyStopState_->to_string()}
        });
        fsm->add_state(this->emergencyStopState_->to_string(), this->emergencyStopState_, {
            {"ES>M", this->manualState_->to_string()},
            {"ES>ET", this->emergencyTakeoverState_->to_string()}
        });

        // execute
        string outcome = fsm->execute(this->blackboard_);
        system("clear");
        cout << "SUPERVISION NODE:\n\n" << outcome << " reached" << endl;
        publishing(this->current_state_pub_, END);
        this_thread::sleep_for(chrono::milliseconds(500));  // timer
        exit(EXIT_SUCCESS);
    }

    // passing SELECTED STATE
    void selected_state_subscription(const std_msgs::msg::String::SharedPtr msg) {
        this->idleState_->set_next_state(msg->data);
        this->manualState_->set_next_state(msg->data);
        this->activeState_->set_next_state(msg->data);
        this->emergencyTakeoverState_->set_next_state(msg->data);
        this->emergencyStopState_->set_next_state(msg->data);
    }

    // passing MANUAL COMMANDS
    void manual_command_subscription(const std_msgs::msg::String::SharedPtr msg) {
        this->manualState_->set_manual_command(msg->data);
    }

    // passing PRIMARY DRIVING STACK
    void primary_driving_stack_subscription(const std_msgs::msg::String::SharedPtr msg) {
        this->activeState_->set_primary_driving_stack_command(msg->data);
    }

    // passing COMMON FAULTS
    void common_fault_subscription(const std_msgs::msg::String::SharedPtr msg) {
        this->activeState_->set_common_fault(msg->data);
    }
};


/******************************************************* Main *********************************************************/
int main(int argc, char *argv[]) {
    cout << "SUPERVISION NODE:\n" << endl;

    // C++ idiomatic interface which provides all the ROS client functionality like creating nodes, publisher, and subscribers
    rclcpp::init(argc, argv);  // activation of rclcpp API
    rclcpp::spin(std::make_shared<SupervisorNode>());  // node creation and spinning
    rclcpp::shutdown();  // shutdown of rclcpp API
    return 0;
}