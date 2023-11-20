#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <random>
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

/// All these times are chosen for the correctness of the simulation
#define INPUT_ERROR_SLEEP 200 // milliseconds
#define UPDATE_CURRENT_STATE_SLEEP 100  // milliseconds
#define END_SLEEP 500  // milliseconds

#define PUB_SUB_QUEUE 1
#define CURRENT_STATE_TOPIC "supervisor_node/current_state"
#define STATE_SELECTION_TOPIC "supervisor_node/state_selection"

using namespace std;

/**************************************************** Methods *********************************************************/
bool input_integer_checker(string &input, int floor, int ceiling) {
    int number;
    std::istringstream iss(input);
    if(iss >> number) {
        if(number >= floor && number <= ceiling) {
            return true;
        } else {
            cerr << "ERROR. Invalid choice selected.\n" << endl;
            return false;
        }
    } else {
        cerr << "ERROR. Input is not an integer.\n" << endl;
        return false;
    }
}

void publisher(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub, const std::string &msg) {
    auto message = std_msgs::msg::String();
    message.data = msg;
    pub->publish(message);
}

void end_execution() {
    system("clear");
    cout << "EXTERNAL STATE SELECTOR NODE:\n\nSUPERVISOR NODE --> OFF" << endl;
    this_thread::sleep_for(chrono::milliseconds(END_SLEEP));  // timer
    exit(EXIT_SUCCESS);
}


/**************************************************** Idle State ******************************************************/
class IdleState : public yasmin::State {
private:
    // parameters
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr idle_state_pub_{};
    string current_state_, state_selection_;

public:
    /// constructor
    IdleState(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub) : yasmin::State({"I>M", "I>End"}),
                                                                          idle_state_pub_(pub) {};
    /// execution
    string execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
        blackboard->set<string>("previous_state", IdleState::to_string());  // memorizing last state

        // state cycle
        do {
            // checking if SUPERVISOR NODE is on
            if (!this->current_state_.empty()) {
                // managing what printing on stdout
                system("clear");
                cout << "EXTERNAL STATE SELECTOR NODE:\n\nSUPERVISOR NODE --> ON\nCurrent state: " << IdleState::to_string() << endl;
                cout << "\nNOTE: if this 'Current state' is not equal to SUPERVISOR NODE's Current state, type one of the following options to reload."
                        "\n\nType the digit of the option you want to choose:" << endl;
                cout << "1. Pass to " + string(END) << endl;
                cout << "2. Pass to " + string(M) << endl;
                cin >> this->state_selection_;  // expecting keyboard input
                cin.ignore();  // cleaning input buffer

                // checking keyboard input correctness
                if (input_integer_checker(this->state_selection_, 1, 2)) {
                    if (this->state_selection_ == "1") {
                        publisher(this->idle_state_pub_, END);
                        return "I>End";
                    } else if (this->state_selection_ == "2") {
                        publisher(this->idle_state_pub_, M);
                        return "I>M";
                    }
                } else {
                    // some time to show error message
                    this_thread::sleep_for(chrono::milliseconds(UPDATE_CURRENT_STATE_SLEEP));
                }

                // expecting to receive last current state
                this_thread::sleep_for(chrono::milliseconds(UPDATE_CURRENT_STATE_SLEEP));  // timer
            }
        } while(true);
    }

    /// other methods
    void set_current_state(string str) {  this->current_state_ = str;  }
    string to_string() {  return I;  }
};


/*************************************************** Manual State *****************************************************/
class ManualState : public yasmin::State {
private:
    // parameters
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr manual_state_pub_{};
    string current_state_, state_selection_;

public:
    /// constructor
    ManualState(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub) : yasmin::State({"M>I", "M>A", "M>ES"}),
                                                                            manual_state_pub_(pub) {};
    /// execution
    string execute(shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
        blackboard->set<string>("previous_state", ManualState::to_string());  // memorizing last state

        // state cycle
        do {
            // managing what printing on stdout
            system("clear");
            cout << "EXTERNAL STATE SELECTOR NODE:\n\nSUPERVISOR NODE --> ON\nCurrent state: " << ManualState::to_string() << endl;
            cout << "\nNOTE: if this 'Current state' is not equal to SUPERVISOR NODE's Current state, type one of the following options to reload."
                    "\n\nType the digit of the option you want to choose:" << endl;
            cout << "1. Pass to " + string(I) << endl;
            cout << "2. Pass to " + string(A) << endl;
            cin >> this->state_selection_;  // expecting keyboard input
            cin.ignore();  // cleaning input buffer

            // checking if current state is changed during the state selection
            if (this->current_state_ != ManualState::to_string()) {
                cout << "Updating to effective CURRENT STATE" << endl;
                this_thread::sleep_for(chrono::milliseconds(UPDATE_CURRENT_STATE_SLEEP));
                if (this->current_state_ == ES) {
                    return "M>ES";
                }
            }

            // checking keyboard input correctness
            if (input_integer_checker(this->state_selection_, 1, 2)) {
                if (this->state_selection_ == "1") {
                    publisher(this->manual_state_pub_, I);
                    return "M>I";
                } else if (this->state_selection_ == "2") {
                    publisher(this->manual_state_pub_, A);
                    return "M>A";
                }
            } else {
                // some time to show error message
                this_thread::sleep_for(chrono::milliseconds(UPDATE_CURRENT_STATE_SLEEP));
            }

            // expecting to receive last current state
            this_thread::sleep_for(chrono::milliseconds(UPDATE_CURRENT_STATE_SLEEP));  // timer
        } while(true);
    }

    /// other methods
    void set_current_state(string str) {  this->current_state_ = str;  }
    string to_string() {  return M;  }
};


/*************************************************** Active State *****************************************************/
class ActiveState : public yasmin::State {
private:
    // parameters
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_state_pub_{};
    string current_state_, state_selection_;

public:
    /// constructor
    ActiveState(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub) : yasmin::State({"A>M", "A>ET", "A>ES"}),
                                                                            active_state_pub_(pub) {};
    /// execution
    string execute(shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
        blackboard->set<string>("previous_state", ActiveState::to_string());  // memorizing last state

        // state cycle
        do {
            // managing what printing on stdout
            system("clear");
            cout << "EXTERNAL STATE SELECTOR NODE:\n\nSUPERVISOR NODE --> ON\nCurrent state: " << ActiveState::to_string() << endl;
            cout << "\nNOTE: if this 'Current state' is not equal to SUPERVISOR NODE's Current state, type one of the following options to reload."
                    "\n\nType the digit of the option you want to choose:" << endl;
            cout << "1. Pass to " + string(M) << endl;
            cin >> this->state_selection_;  // expecting keyboard input
            cin.ignore();  // cleaning input buffer

            // checking if current state is changed during the state selection
            if (this->current_state_ != ActiveState::to_string()) {
                cout << "Updating to effective CURRENT STATE" << endl;
                this_thread::sleep_for(chrono::milliseconds(UPDATE_CURRENT_STATE_SLEEP));
                if (this->current_state_ == ET) {
                    return "A>ET";
                } else if (this->current_state_ == ES) {
                    return "A>ES";
                }
            }

            // checking keyboard input correctness
            if (input_integer_checker(this->state_selection_, 1, 1)) {
                if (this->state_selection_ == "1") {
                    publisher(this->active_state_pub_, M);
                    return "A>M";
                }
            } else {
                // some time to show error message
                this_thread::sleep_for(chrono::milliseconds(UPDATE_CURRENT_STATE_SLEEP));
            }

            // expecting to receive last current state
            this_thread::sleep_for(chrono::milliseconds(UPDATE_CURRENT_STATE_SLEEP));  // timer
        } while(true);
    }

    /// other methods
    void set_current_state(string str) {  this->current_state_ = str;  }
    string to_string() {  return A;  }
};


/********************************************* Emergency Takeover State ***********************************************/
class EmergencyTakeoverState : public yasmin::State {
private:
    // parameters
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr emergency_takeover_state_pub_{};
    string current_state_, state_selection_;

public:
    /// constructor
    EmergencyTakeoverState(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub) : yasmin::State({"ET>M", "ET>A", "ET>ES"}),
                                                                                       emergency_takeover_state_pub_(pub) {};
    /// execution
    string execute(shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
        blackboard->set<string>("previous_state", EmergencyTakeoverState::to_string());  // memorizing last state

        // state cycle
        do {
            // managing what printing on stdout
            system("clear");
            cout << "EXTERNAL STATE SELECTOR NODE:\n\nSUPERVISOR NODE --> ON\nCurrent state: " << EmergencyTakeoverState::to_string() << endl;
            cout << "\nNOTE: if this 'Current state' is not equal to SUPERVISOR NODE's Current state, type one of the following options to reload."
                    "\n\nType the digit of the option you want to choose:" << endl;
            cout << "1. Pass to " + string(M) << endl;
            cin >> this->state_selection_;  // expecting keyboard input
            cin.ignore();  // cleaning input buffer

            // checking if current state is changed during the state selection
            if (this->current_state_ != EmergencyTakeoverState::to_string()) {
                cout << "Updating to effective CURRENT STATE" << endl;
                this_thread::sleep_for(chrono::milliseconds(UPDATE_CURRENT_STATE_SLEEP));
                if (this->current_state_ == A) {
                    return "ET>A";
                } else if (this->current_state_ == ES) {
                    return "ET>ES";  // --> the serious fault is resolved
                }
            }

            // checking keyboard input correctness
            if (input_integer_checker(this->state_selection_, 1, 1)) {
                if (this->state_selection_ == "1") {
                    publisher(this->emergency_takeover_state_pub_, M);
                    return "ET>M";  // --> the common fault is resolved
                }
            } else {
                // some time to show error message
                this_thread::sleep_for(chrono::milliseconds(UPDATE_CURRENT_STATE_SLEEP));
            }

            // expecting to receive last current state
            this_thread::sleep_for(chrono::milliseconds(UPDATE_CURRENT_STATE_SLEEP));  // timer
        } while(true);
    }

    /// other methods
    void set_current_state(string str) {  this->current_state_ = str;  }
    string to_string() {  return ET;  }
};


/*********************************************** Emergency Stop State *************************************************/
class EmergencyStopState : public yasmin::State {
private:
    // parameters
    string current_state_;

public:
    /// constructor
    EmergencyStopState() : yasmin::State({"ES>M", "ES>ET"}) {};

    /// methods
    string execute(shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
        blackboard->set<string>("previous_state", EmergencyStopState::to_string());  // memorizing last state

        // state cycle
        do {
            // managing what printing on stdout
            system("clear");
            cout << "EXTERNAL STATE SELECTOR NODE:\n\nSUPERVISOR NODE --> ON\nCurrent state: " << EmergencyStopState::to_string() << endl;
            cout << "\nNecessary waiting until the serious fault is resolved." << endl;

            // checking if current state is changed
            if (this->current_state_ == M) {
                return "ES>M";  // --> the serious fault is resolved
            } else if (this->current_state_ == ET) {
                return "ES>ET";  // --> the serious fault is resolved
            }

            // expecting to receive last current state
            this_thread::sleep_for(chrono::milliseconds(UPDATE_CURRENT_STATE_SLEEP));  // timer
        } while(true);
    }

    /// other methods
    void set_current_state(string str) {  this->current_state_ = str;  }
    string to_string() {  return ES;  }
};


/******************************************** External State Selector Node ********************************************/
class ExternalStateSelectorNode : public simple_node::Node {
private:
    // blackboard
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard_ = std::make_shared<yasmin::blackboard::Blackboard>();

    // publishers and subscribers
    /// CURRENT STATE SUBSCRIPTION
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr current_state_subscription_ = this->create_subscription<std_msgs::msg::String>(
            CURRENT_STATE_TOPIC,
            rclcpp::QoS(rclcpp::KeepLast(PUB_SUB_QUEUE)).reliable().transient_local(),
            std::bind(&ExternalStateSelectorNode::current_state_subscription, this, placeholders::_1)
    );
    /// STATE SELECTION PUBLISHER
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publish_state_selection_ = this->create_publisher<std_msgs::msg::String>(
            STATE_SELECTION_TOPIC,
            rclcpp::QoS(rclcpp::KeepLast(PUB_SUB_QUEUE)).reliable()
    );

    // states
    std::shared_ptr<IdleState> idleState_ = std::make_shared<IdleState>(this->publish_state_selection_);
    std::shared_ptr<ManualState> manualState_ = std::make_shared<ManualState>(this->publish_state_selection_);
    std::shared_ptr<ActiveState> activeState_ = std::make_shared<ActiveState>(this->publish_state_selection_);
    std::shared_ptr<EmergencyTakeoverState> emergencyTakeoverState_ = std::make_shared<EmergencyTakeoverState>(this->publish_state_selection_);
    std::shared_ptr<EmergencyStopState> emergencyStopState_ = std::make_shared<EmergencyStopState>();

public:
    /// constructor
    ExternalStateSelectorNode() : simple_node::Node("external_state_selector_node") {

        // create a finite state machine
        auto fsm = std::make_shared<yasmin::StateMachine>(yasmin::StateMachine({END}));

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
        fsm->execute(this->blackboard_);

        // exit
        end_execution();
    }

    /// other methods
    // passing SUPERVISOR NODE CURRENT STATE
    void current_state_subscription(const std_msgs::msg::String::SharedPtr msg) {
        this->idleState_->set_current_state(msg->data);
        this->manualState_->set_current_state(msg->data);
        this->activeState_->set_current_state(msg->data);
        this->emergencyTakeoverState_->set_current_state(msg->data);
        this->emergencyStopState_->set_current_state(msg->data);
    }
};


/******************************************************* Main *********************************************************/
int main(int argc, char * argv[]) {
    // managing what printing on stdout
    cout << "EXTERNAL STATE SELECTOR NODE:\n\nSUPERVISOR NODE --> OFF\n" << endl;

    // C++ idiomatic interface which provides all the ROS client functionality like creating nodes, publisher, and subscribers
    rclcpp::init(argc, argv);  // activation of rclcpp API
    rclcpp::spin(std::make_shared<ExternalStateSelectorNode>()); // node creation and spinning
    rclcpp::shutdown();  // shutdown of rclcpp API
    return 0;
}