#ifndef SUPERVISOR_NODE_ACTIVE_H
#define SUPERVISOR_NODE_ACTIVE_H

#include "../header/header.h"

/*************************************************** Active State *****************************************************/
class ActiveState : public yasmin::State {
private:
    // parameters
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr primary_driving_stack_publisher_{};
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr common_fault_publisher_{};
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr general_driver_response_publisher_{};
    string selected_state_, current_state_;
    thread primary_driving_stack_thread_, common_fault_thread_, general_driver_response_thread_;

public:
    /// constructor
    ActiveState(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub1,
                rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub2,
                rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub3) : yasmin::State({"A>M", "A>ET", "A>ES"}),
                                                                             primary_driving_stack_publisher_(pub1),
                                                                             common_fault_publisher_(pub2),
                                                                             general_driver_response_publisher_(pub3) {};
    /// execution
    string execute(shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
        blackboard->set<string>("previous_state", ActiveState::to_string());  // memorizing last state

        // starting publishers threads
        this->primary_driving_stack_thread_ = thread([this]() {  // opening new simulation thread
            polling_publisher(this->primary_driving_stack_publisher_, ActiveState::to_string(), PRIMARY_DRIVING_STACK_PUBLISHER_SLEEP);
        });
        this->common_fault_thread_ = thread([this]() {  // opening new simulation thread
            polling_publisher(this->common_fault_publisher_, ActiveState::to_string(), PRIMARY_DRIVING_STACK_PUBLISHER_SLEEP, 1);
        });
        this->general_driver_response_thread_ = thread([this]() {  // opening new simulation thread
            polling_publisher(this->general_driver_response_publisher_, ActiveState::to_string(), GENERAL_DRIVER_RESPONSE_SLEEP, 2);
        });

        // state cycle
        do {
            // timer
            this_thread::sleep_for(chrono::milliseconds(ACTIVE_SLEEP));

            // managing transitions
            if (this->current_state_ == ET) {  // checking if SUPERVISOR NODE has switched to EMERGENCY TAKEOVER state
                this->exit_publisher_thread();
                return "A>ET";
            } else if (this->current_state_ == ES) {  // checking if SUPERVISOR NODE has switched to EMERGENCY STOP state
                this->exit_publisher_thread();
                return "A>ES";
            } else if (this->selected_state_ == M) {  // checking if SUPERVISOR NODE has switched to MANUAL state
                this->exit_publisher_thread();
                return "A>M";
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
        this->common_fault_thread_.join();
        this->general_driver_response_thread_.join();
    }

    void set_selected_state(string str) {
        this->selected_state_ = str;
    }

    void set_current_state(string str) {
        this->current_state_ = str;
    }

    string to_string() {
        return A;
    }
};

#endif //SUPERVISOR_NODE_ACTIVE_H