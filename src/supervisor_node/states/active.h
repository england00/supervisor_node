#ifndef SUPERVISOR_NODE_ACTIVE_STATE_H
#define SUPERVISOR_NODE_ACTIVE_STATE_H

#include "../header/supervisor_node_header.h"

/*************************************************** Active State *****************************************************/
class ActiveState : public yasmin::State {
private:
    // parameters
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_state_pub_{};
    string next_state_, primary_driving_stack_command_, common_fault_, general_driver_response_;
    bool deadline_missed_, lost_liveliness_;

public:
    /// constructor
    ActiveState(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub) : yasmin::State({"A>M", "A>ET", "A>ES"}),
                                                                            active_state_pub_(pub) {};
    /// execution
    string execute(shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
        publishing(active_state_pub_, ActiveState::to_string());
        string stdout_message = "Executing state " + ActiveState::to_string() + "\n\n"
                                                                                "The vehicle is in autonomous driving mode, therefore:\n"
                                                                                "- fault checks are performed;\n"
                                                                                "- control is entrusted to the primary driving stack.";
        string stdout_buffer;
        this->deadline_missed_ = false;
        this->common_fault_.clear();
        this->lost_liveliness_ = false;
        if (blackboard->get<string>("previous_state") == ET)  // optional timer
            this_thread::sleep_for(chrono::milliseconds(END_SLEEP));
        blackboard->set<string>("previous_state", ActiveState::to_string());  // memorizing last state

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
            if (this->deadline_missed_) {  /// DEADLINE MISSED
                blackboard->set<string>("error_cause_ET", timestamp("DEADLINE MISSED", 1));
                return "A>ET";
            } else if (!this->common_fault_.empty()) {  /// COMMON FAULT
                blackboard->set<string>("error_cause_ET", timestamp(this->common_fault_, 1));
                return "A>ET";
            } else if (this->lost_liveliness_) {  /// LOST LIVELINESS
                blackboard->set<string>("error_cause_ET", timestamp("recovering from LOST LIVELINESS", 1));
                blackboard->set<string>("error_cause_ES", timestamp("LOST LIVELINESS", 1));
                return "A>ES";
            } else if (this->next_state_ == M) {  // checking if MANUAL state has been selected
                return "A>M";
            }
        } while(true);
    }

    /// other methods
    void set_next_state(string str) {  this->next_state_ = str;  }
    void set_primary_driving_stack_command(string str) {  this->primary_driving_stack_command_ = str;  }
    void set_deadline_missed() {  this->deadline_missed_ = true;  }
    void set_common_fault(string str) {  this->common_fault_ = str;  }
    void set_general_driver_response(string str) {  this->general_driver_response_ = str;  }
    void set_lost_liveliness() {  this->lost_liveliness_ = true;  }
    string to_string() {  return A;  }
};

#endif //SUPERVISOR_NODE_ACTIVE_STATE_H