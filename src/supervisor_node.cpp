#include <chrono>
#include <ctime>
#include <iostream>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
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
#define END_SLEEP 500  // milliseconds
#define EMERGENCY_TIMER 50000
#define DEADLINE_PRIMARY_STACK 200  // milliseconds
#define DEADLINE_SECONDARY_STACK 300  // milliseconds
#define CURRENT_STATE_TOPIC "supervisor_node/current_state"
#define STATE_SELECTION_TOPIC "supervisor_node/state_selection"
#define MANUAL_COMMAND_TOPIC "supervisor_node/manual_command"
#define PRIMARY_DRIVING_STACK_TOPIC "supervisor_node/primary_driving_stack"
#define SECONDARY_DRIVING_STACK_TOPIC "supervisor_node/secondary_driving_stack"
#define COMMON_FAULT_TOPIC "supervisor_node/common_fault"
#define SERIOUS_FAULT_TOPIC "supervisor_node/serious_fault"
#define GENERAL_DRIVER_TOPIC "supervisor_node/general_sensor_or_actuator_driver_response"

using namespace std;

/***************************************************** Methods ********************************************************/
void publishing(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub, const std::string &msg) {
    auto message = std_msgs::msg::String();
    message.data = msg;
    pub->publish(message);
}

string timestamp(string command, int print_type = 0) {
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

    if (print_type == 0) {
        return "\nReceived message on " + std::string(time_buffer) + ":\n --> " + command;
    } else if (print_type == 1) {
        return "\nEmergency on " + std::string(time_buffer) + ":\n --> " + command;
    } else if (print_type == 2) {
        return "\nResolving " + command + " on " + std::string(time_buffer);
    }

}

void end_execution(string outcome) {
    system("clear");
    cout << "SUPERVISION NODE:\n\n" << outcome << " reached" << endl;
    this_thread::sleep_for(chrono::milliseconds(END_SLEEP));  // timer
    exit(EXIT_SUCCESS);
}


/**************************************************** Idle State ******************************************************/
class IdleState : public yasmin::State {
private:
    // parameters
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr idle_state_pub_{};
    string next_state_;

public:
    /// constructor
    IdleState(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub) : yasmin::State({"I>M", "I>End"}),
                                                                          idle_state_pub_(pub) {};
    /// execution
    string execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
        string stdout_message = "Executing state " + IdleState::to_string() + "\n\n"
                                "The node is on and awaits signals from the outside.";
        this_thread::sleep_for(chrono::milliseconds(GENERAL_SLEEP));  // timer
        blackboard->set<string>("previous_state", IdleState::to_string());  // memorizing last state
        publishing(idle_state_pub_, IdleState::to_string());

        // state cycle
        do {
            // managing what printing on stdout
            system("clear");
            cout << "SUPERVISION NODE:\n" << endl;
            cout << stdout_message << endl;
            this_thread::sleep_for(chrono::milliseconds(GENERAL_SLEEP));  // timer

            // managing transitions
            if (this->next_state_ == M) {  // checking if MANUAL state has been selected
                return "I>M";
            } else if (this->next_state_ == END) {  // checking if END state has been selected
                return "I>End";
            }
        } while(true);
    }

    /// other methods
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
    /// constructor
    ManualState(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub) : yasmin::State({"M>I", "M>A", "M>ES"}),
                                                                            manual_state_pub_(pub) {};
    /// execution
    string execute(shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
        string stdout_message = "Executing state " + ManualState::to_string() + "\n\n"
                                "The vehicle is in manual driving mode, therefore:\n"
                                "- no fault checks are performed;\n"
                                "- all control commands provided by the primary and secondary stack are ignored.";
        string stdout_buffer;
        this_thread::sleep_for(chrono::milliseconds(GENERAL_SLEEP));  // timer
        blackboard->set<string>("previous_state", ManualState::to_string());  // memorizing last state
        publishing(manual_state_pub_, ManualState::to_string());

        // state cycle
        do {
            // managing what printing on stdout
            system("clear");
            cout << "SUPERVISION NODE:\n" << endl;
            cout << stdout_message << endl;  // default output
            if (!this->manual_command_.empty()) {  // adding the last command received
                stdout_buffer = timestamp(this->manual_command_);
            }
            cout << stdout_buffer << endl;
            this->manual_command_ = "";  // cleaning latest command buffer
            this_thread::sleep_for(chrono::milliseconds(GENERAL_SLEEP));  // timer

            // managing transitions
            if (this->next_state_ == I) {  // checking if IDLE state has been selected
                return "M>I";
            } else if (this->next_state_ == A) {  // checking if ACTIVE state has been selected
                return "M>A";
            }
        } while(true);
    }

    /// other methods
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
    bool deadline_missed;

public:
    /// constructor
    ActiveState(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub) : yasmin::State({"A>M", "A>ET", "A>ES"}),
                                                                            active_state_pub_(pub) {};
    /// execution
    string execute(shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
        string stdout_message = "Executing state " + ActiveState::to_string() + "\n\n"
                                "The vehicle is in autonomous driving mode, therefore:\n"
                                "- fault checks are performed;\n"
                                "- control is entrusted to the primary driving stack.";
        string stdout_buffer;
        this->deadline_missed = false;
        this->common_fault_.clear();
        if (blackboard->get<string>("previous_state") == ET)  // optional timer
            this_thread::sleep_for(chrono::milliseconds(END_SLEEP));
        blackboard->set<string>("previous_state", ActiveState::to_string());  // memorizing last state
        publishing(active_state_pub_, ActiveState::to_string());

        // state cycle
        do {
            // managing what printing on stdout
            system("clear");
            cout << "SUPERVISION NODE:\n" << endl;
            cout << stdout_message << endl;  // default output
            if (!this->primary_driving_stack_command_.empty()) {  // adding the last command received
                stdout_buffer = timestamp(this->primary_driving_stack_command_);
            }
            cout << stdout_buffer << endl;
            this->primary_driving_stack_command_ = "";  // cleaning latest command buffer
            this_thread::sleep_for(chrono::milliseconds(GENERAL_SLEEP));  // timer

            // managing transitions
            if (this->deadline_missed) {  /// DEADLINE MISSED
                blackboard->set<string>("error_cause", timestamp("DEADLINE MISSED", 1));
                return "A>ET";
            }
            if (!this->common_fault_.empty()) {  /// COMMON FAULT
                blackboard->set<string>("error_cause", timestamp(this->common_fault_, 1));
                return "A>ET";
            }
            if (this->next_state_ == M) {  // checking if MANUAL state has been selected
                return "A>M";
            }
        } while(true);
    }

    /// other methods
    void set_next_state(string str) {  this->next_state_ = str;  }
    void set_primary_driving_stack_command(string str) {  this->primary_driving_stack_command_ = str;  }
    void set_deadline_missed() {  this->deadline_missed = true;  }
    void set_common_fault(string str) {  this->common_fault_ = str;  }
    string to_string() {  return A;  }
};


/********************************************* Emergency Takeover State ***********************************************/
class EmergencyTakeoverState : public yasmin::State {
private:
    // parameters
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr emergency_takeover_state_pub_{};
    string next_state_, secondary_driving_stack_command_;
    bool deadline_missed;

public:
    /// constructor
    EmergencyTakeoverState(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub) : yasmin::State({"ET>M", "ET>A", "ET>ES"}),
                                                                                       emergency_takeover_state_pub_(pub) {};
    /// execution
    string execute(shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
        string stdout_message = "Executing state " + EmergencyTakeoverState::to_string() + "\n\n"
                                "The vehicle is in a state of risk:\n"
                                "- control is entrusted to the secondary driving stack.";
        string stdout_buffer;
        this->deadline_missed = false;
        if (blackboard->get<string>("previous_state") == ES)  // optional timer
            this_thread::sleep_for(chrono::milliseconds(END_SLEEP));
        blackboard->set<string>("previous_state", EmergencyTakeoverState::to_string());  // memorizing last state
        clock_t begin = clock();  // timer start

        // state cycle
        do {
            // publishing current state in each iteration
            publishing(emergency_takeover_state_pub_, EmergencyTakeoverState::to_string());

            // managing what printing on stdout
            system("clear");
            cout << "SUPERVISION NODE:\n" << endl;
            cout << stdout_message << endl;  // default output
            cout << blackboard->get<string>("error_cause") << endl;  // emergency type
            stdout_buffer = timestamp("COMMON FAULT", 2);
            if (!this->secondary_driving_stack_command_.empty()) {  // adding the last command received
                stdout_buffer += "\n" + timestamp(this->secondary_driving_stack_command_);
            }
            cout << stdout_buffer << endl;
            this->secondary_driving_stack_command_ = "";  // cleaning latest command buffer
            this_thread::sleep_for(chrono::milliseconds(GENERAL_SLEEP));  // timer

            // managing transition
            if (this->deadline_missed) {  /// DEADLINE MISSED
                blackboard->set<string>("error_cause", timestamp("DEADLINE MISSED", 1));
                return "ET>ES";
            }
            if (this->next_state_ == M) {  // checking if MANUAL state has been selected
                return "ET>M";
            }
            if ((clock() - begin) > EMERGENCY_TIMER) {  /// return to PREVIOUS STATE
                return "ET>A";  // --> the common fault is resolved
            }
        } while(true);
    }

    /// other methods
    void set_next_state(string str) {  this->next_state_ = str;  }
    void set_secondary_driving_stack_command(string str) {  this->secondary_driving_stack_command_ = str;  }
    void set_deadline_missed() {  this->deadline_missed = true;  }
    string to_string() {  return ET;  }
};


/*********************************************** Emergency Stop State *************************************************/
class EmergencyStopState : public yasmin::State {
private:
    // parameters
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr emergency_stop_state_pub_{};
    string next_state_, previous_state_;

public:
    /// constructor
    EmergencyStopState(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub) : yasmin::State({"ES>M", "ES>ET"}),
                                                                                   emergency_stop_state_pub_(pub) {};
    /// methods
    string execute(shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
        string stdout_message = "Executing state " + EmergencyStopState::to_string() + "\n\n"
                                "The vehicle is unable to move autonomously:\n"
                                "- driving commands are ignored and the vehicle is brought to a stop.";
        string stdout_buffer;
        this->previous_state_ = blackboard->get<string>("previous_state");
        blackboard->set<string>("previous_state", EmergencyStopState::to_string());  // memorizing last state
        clock_t begin = clock();  // timer start

        // state cycle
        do {
            // publishing current state in each iteration
            publishing(emergency_stop_state_pub_, EmergencyStopState::to_string());

            // managing what printing on stdout
            system("clear");
            cout << "SUPERVISION NODE:\n" << endl;
            cout << stdout_message << endl;  // default output
            cout << blackboard->get<string>("error_cause") << endl;  // emergency type
            stdout_buffer = timestamp("SERIOUS FAULT", 2);
            cout << stdout_buffer << endl;
            this_thread::sleep_for(chrono::milliseconds(GENERAL_SLEEP));  // timer

            // managing transition
            if ((clock() - begin) > EMERGENCY_TIMER and this->previous_state_ == M) {   /// return to PREVIOUS STATE
                return "ES>M";  // --> the serious fault is resolved
            } else if ((clock() - begin) > EMERGENCY_TIMER and (this->previous_state_ == A || this->previous_state_ == ET)) {  /// return to PREVIOUS STATE
                return "ES>ET";  // --> the serious fault is resolved
            }
        } while(true);
    }

    /// other methods
    void set_next_state(string str) {  this->next_state_ = str;  }
    string to_string() {  return ES;  }
};


/************************************************** Supervisor Node ***************************************************/
class SupervisorNode : public simple_node::Node {
private:
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard_ = std::make_shared<yasmin::blackboard::Blackboard>();

    // publishers and subscribers
    /// CURRENT STATE PUBLISHER
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr current_state_pub_ = this->create_publisher<std_msgs::msg::String>(
        CURRENT_STATE_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local()
    );
    /// SELECTED STATE SUBSCRIPTION
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr selected_state_sub_ = this->create_subscription<std_msgs::msg::String>(
        STATE_SELECTION_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
        std::bind(&SupervisorNode::selected_state_subscription, this, placeholders::_1)
    );
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr manual_command_sub_ = nullptr;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr common_fault_sub_ = nullptr;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr primary_driving_stack_sub_ = nullptr;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr secondary_driving_stack_sub_ = nullptr;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr general_driver_response_sub_ = nullptr;
    std::unique_ptr<yasmin_viewer::YasminViewerPub> yasmin_pub_{};

    // states
    std::shared_ptr<IdleState> idleState_ = std::make_shared<IdleState>(this->current_state_pub_);
    std::shared_ptr<ManualState> manualState_ = std::make_shared<ManualState>(this->current_state_pub_);
    std::shared_ptr<ActiveState> activeState_ = std::make_shared<ActiveState>(this->current_state_pub_);
    std::shared_ptr<EmergencyTakeoverState> emergencyTakeoverState_ = std::make_shared<EmergencyTakeoverState>(this->current_state_pub_);
    std::shared_ptr<EmergencyStopState> emergencyStopState_ = std::make_shared<EmergencyStopState>(this->current_state_pub_);

public:
    /// constructor
    SupervisorNode() : simple_node::Node("supervisor_node") {

        /// MANUAL COMMANDS SUBSCRIPTION
        rclcpp::QoS qos_profile(rclcpp::KeepLast(1));  // queue dimension
        qos_profile.reliable();  // type of communication
        this->manual_command_sub_ = this->create_subscription<std_msgs::msg::String>(MANUAL_COMMAND_TOPIC, qos_profile,
            std::bind(&SupervisorNode::manual_command_subscription, this, placeholders::_1)
        );
        /// COMMON FAULTS SUBSCRIPTION
        this->common_fault_sub_ = this->create_subscription<std_msgs::msg::String>(COMMON_FAULT_TOPIC, qos_profile,
            std::bind(&SupervisorNode::common_fault_subscription, this, placeholders::_1)
        );
        /// PRIMARY DRIVING STACK SUBSCRIPTION
        qos_profile.deadline(chrono::milliseconds(DEADLINE_PRIMARY_STACK));  // imposing deadline
        rclcpp::SubscriptionOptions subscription_options_primary_driving_stack;
        subscription_options_primary_driving_stack.event_callbacks.deadline_callback = [this](rclcpp::QOSDeadlineRequestedInfo &event) -> void {
            if (this->blackboard_->get<string>("previous_state") == A)  // only if SUPERVISOR NODE is in ACTIVE state
                this->activeState_->set_deadline_missed();  // managing MISSED DEADLINE
        };
        this->primary_driving_stack_sub_ = this->create_subscription<std_msgs::msg::String>(PRIMARY_DRIVING_STACK_TOPIC,
            qos_profile,
            std::bind(&SupervisorNode::primary_driving_stack_subscription, this, placeholders::_1),
            subscription_options_primary_driving_stack
        );
        /// SECONDARY DRIVING STACK SUBSCRIPTION
        qos_profile.deadline(chrono::milliseconds(DEADLINE_SECONDARY_STACK));
        rclcpp::SubscriptionOptions subscription_options_secondary_driving_stack;
        subscription_options_secondary_driving_stack.event_callbacks.deadline_callback = [this](rclcpp::QOSDeadlineRequestedInfo &event) -> void {
            if (this->blackboard_->get<string>("previous_state") == ET)  // only if SUPERVISOR NODE is in EMERGENCY TAKEOVER state
                this->emergencyTakeoverState_->set_deadline_missed();  // managing MISSED DEADLINE
        };
        this->secondary_driving_stack_sub_ = this->create_subscription<std_msgs::msg::String>(SECONDARY_DRIVING_STACK_TOPIC,
            qos_profile,
            std::bind(&SupervisorNode::secondary_driving_stack_subscription, this, placeholders::_1),
            subscription_options_secondary_driving_stack
        );

        /*
        subscription_options.event_callbacks.liveliness_callback = [this](rclcpp::QOSLivelinessChangedInfo &event) -> void {
            // managing CHANGED LIVELINESS
            system("clear");
            cout << "Liveliness changed - alive " << event.alive_count << " (delta " << event.alive_count_change << "), "
                    "not alive " << event.not_alive_count << " (delta " << event.not_alive_count_change << ")" << endl;
        };*/
        //qos_profile.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC).liveliness_lease_duration(chrono::milliseconds(2000));

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

        // executing fsm
        string outcome = fsm->execute(this->blackboard_);

        // exit
        publishing(this->current_state_pub_, END);
        end_execution(outcome);
    }

    /// other methods
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
    // passing SECONDARY DRIVING STACK
    void secondary_driving_stack_subscription(const std_msgs::msg::String::SharedPtr msg) {
        this->emergencyTakeoverState_->set_secondary_driving_stack_command(msg->data);
    }
    // passing COMMON FAULTS
    void common_fault_subscription(const std_msgs::msg::String::SharedPtr msg) {
        this->activeState_->set_common_fault(msg->data);
    }
};


/******************************************************* Main *********************************************************/
int main(int argc, char *argv[]) {
    // managing what printing on stdout
    cout << "SUPERVISION NODE:\n" << endl;

    // C++ idiomatic interface which provides all the ROS client functionality like creating nodes, publisher, and subscribers
    rclcpp::init(argc, argv);  // activation of rclcpp API
    rclcpp::spin(std::make_shared<SupervisorNode>());  // node creation and spinning
    rclcpp::shutdown();  // shutdown of rclcpp API
    return 0;
}