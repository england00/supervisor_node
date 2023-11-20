#ifndef SUPERVISOR_NODE_MANUAL_H
#define SUPERVISOR_NODE_MANUAL_H

#include "../header/header.h"

/*************************************************** Manual State *****************************************************/
class ManualState : public yasmin::State {
private:
    // parameters
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr manual_commands_publisher_{};
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr general_driver_response_publisher_{};
    string selected_state_, current_state_;
    thread manual_commands_thread_, general_driver_response_thread_;

public:
    /// constructor
    ManualState(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub1,
                rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub2) : yasmin::State({"M>I", "M>A", "M>ES"}),
                                                                             manual_commands_publisher_(pub1),
                                                                             general_driver_response_publisher_(pub2) {};
    /// execution
    string execute(shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
        blackboard->set<string>("previous_state", ManualState::to_string());  // memorizing last state

        // starting publishers threads
        this->manual_commands_thread_ = thread([this]() {  // opening new simulation thread
            polling_publisher(this->manual_commands_publisher_, ManualState::to_string(), MANUAL_COMMAND_PUBLISHER_SLEEP);
        });
        this->general_driver_response_thread_ = thread([this]() {  // opening new simulation thread
            polling_publisher(this->general_driver_response_publisher_, ManualState::to_string(), GENERAL_DRIVER_RESPONSE_SLEEP, 2);
        });

        // state cycle
        do {
            // timer
            this_thread::sleep_for(chrono::milliseconds(GENERAL_SLEEP));

            // managing transitions
            if (this->current_state_ == ES) {  // checking if SUPERVISOR NODE has switched to EMERGENCY STOP state
                this->exit_publisher_thread();
                return "M>ES";
            } else if (this->selected_state_ == I) {  // checking if SUPERVISOR NODE has switched to IDLE state
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
        this->general_driver_response_thread_.join();
    }

    void set_selected_state(string str) {
        this->selected_state_ = str;
    }

    void set_current_state(string str) {
        this->current_state_ = str;
    }

    string to_string() {
        return M;
    }
};

#endif //SUPERVISOR_NODE_MANUAL_H