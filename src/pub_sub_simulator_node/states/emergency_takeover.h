#ifndef SUPERVISOR_NODE_EMERGENCY_TAKEOVER_H
#define SUPERVISOR_NODE_EMERGENCY_TAKEOVER_H

#include "../header/header.h"

/********************************************* Emergency Takeover State ***********************************************/
class EmergencyTakeoverState : public yasmin::State {
private:
    // parameters
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr secondary_driving_stack_publisher_{};
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr general_driver_response_publisher_{};
    string selected_state_, current_state_;
    thread secondary_driving_stack_thread_, general_driver_response_thread_;

public:
    /// constructor
    EmergencyTakeoverState(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub1,
                           rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub2) : yasmin::State({"ET>M", "ET>A", "ET>ES"}),
                                                                                        secondary_driving_stack_publisher_(pub1),
                                                                                        general_driver_response_publisher_(pub2) {};
    /// execution
    string execute(shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
        blackboard->set<string>("previous_state", EmergencyTakeoverState::to_string());  // memorizing last state

        // starting publishers threads
        this->secondary_driving_stack_thread_ = thread([this]() {  // opening new simulation thread
            polling_publisher(this->secondary_driving_stack_publisher_, EmergencyTakeoverState::to_string(), SECONDARY_DRIVING_STACK_PUBLISHER_SLEEP);
        });
        this->general_driver_response_thread_ = thread([this]() {  // opening new simulation thread
            polling_publisher(this->general_driver_response_publisher_, EmergencyTakeoverState::to_string(), GENERAL_DRIVER_RESPONSE_SLEEP, 2);
        });

        // state cycle
        do {
            // timer
            this_thread::sleep_for(chrono::milliseconds(ACTIVE_SLEEP));

            // managing transitions
            if (this->current_state_ == A) {  // checking if SUPERVISOR NODE has switched to ACTIVE state
                this->exit_publisher_thread();
                return "ET>A";
            } else if (this->current_state_ == ES) {  // checking if SUPERVISOR NODE has switched to EMERGENCY STOP state
                this->exit_publisher_thread();
                return "ET>ES";
            } else if (this->selected_state_ == M) {  // checking if SUPERVISOR NODE has switched to MANUAL state
                this->exit_publisher_thread();
                return "ET>M";
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
        this->general_driver_response_thread_.join();
    }

    void set_selected_state(string str) {
        this->selected_state_ = str;
    }

    void set_current_state(string str) {
        this->current_state_ = str;
    }

    string to_string() {
        return ET;
    }
};

#endif //SUPERVISOR_NODE_EMERGENCY_TAKEOVER_H