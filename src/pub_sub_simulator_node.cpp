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


/**************************************************** Idle State ******************************************************/
class IdleState : public yasmin::State {
private:
    // parameters
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr idle_state_pub_{};
    string current_state_ = I;

public:
    // constructor
    IdleState(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub) : yasmin::State({"I>M", "I>End"}),
                                                                          idle_state_pub_(pub) {};
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
            this_thread::sleep_for(chrono::milliseconds(SLEEP));  // timer

            // managing transitions
            if (this->current_state_ == M) {  // checking if SUPERVISOR NODE has switched to MANUAL state
                return "I>M";
            } else if (this->current_state_ == END) {  // checking if SUPERVISOR NODE has switched to END state
                return "I>End";
            }
        } while(true);
    }

    void set_current_state(string str) {  this->current_state_ = str;  }
    string to_string() {  return I;  }
};


/*************************************************** Manual State *****************************************************/
class ManualState : public yasmin::State {
private:
    // parameters
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr manual_state_pub_{};
    string current_state_ = M;

public:
    // constructor
    ManualState(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub) : yasmin::State({"M>I", "M>A", "M>ES"}),
                                                                            manual_state_pub_(pub) {};
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
                    this->current_state_ << "\n\n\"Publishing manual commands." << endl;
            this_thread::sleep_for(chrono::milliseconds(SLEEP));  // timer

            // managing transitions
            if (this->current_state_ == I) {  // checking if SUPERVISOR NODE has switched to IDLE state
                return "M>I";
            } else if (this->current_state_ == A) {  // checking if SUPERVISOR NODE has switched to ACTIVE state
                return "M>A";
            }
        } while(true);
    }

    void set_current_state(string str) {  this->current_state_ = str;  }
    string to_string() {  return M;  }
};


/*************************************************** Active State *****************************************************/
class ActiveState : public yasmin::State {
private:
    // parameters
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_state_pub_{};
    string current_state_ = A;

public:
    // constructor
    ActiveState(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub) : yasmin::State({"A>M", "A>ET", "A>ES"}),
                                                                            active_state_pub_(pub) {};
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
                 this->current_state_ << "\n\n\"Publishing primary driving stack." << endl;
            this_thread::sleep_for(chrono::milliseconds(SLEEP));  // timer

            // managing transitions
            if (this->current_state_ == M) {  // checking if SUPERVISOR NODE has switched to MANUAL state
                return "A>M";
            }
        } while(true);
    }

    void set_current_state(string str) {  this->current_state_ = str;  }
    string to_string() {  return A;  }
};


/********************************************** Pub/Sub Simulator Node ************************************************/
class PubSubSimulatorNode : public simple_node::Node {
private:
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard_ = std::make_shared<yasmin::blackboard::Blackboard>();

    // publishers and subscribers
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr current_state_subscription_ = this->create_subscription<std_msgs::msg::String>(
            CURRENT_STATE_TOPIC,
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
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
    std::shared_ptr<IdleState> idleState_ = std::make_shared<IdleState>(this->publish_primary_driving_stack_command_);
    std::shared_ptr<ManualState> manualState_ = std::make_shared<ManualState>(this->publish_primary_driving_stack_command_);
    std::shared_ptr<ActiveState> activeState_ = std::make_shared<ActiveState>(this->publish_primary_driving_stack_command_);

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

        // executing fsm
        fsm->execute(this->blackboard_);

        // managing what printing on stdout
        system("clear");
        cout << "PUB/SUB SIMULATOR NODE:\n\nSUPERVISOR NODE --> OFF" << endl;
        this_thread::sleep_for(chrono::milliseconds(500));  // timer
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