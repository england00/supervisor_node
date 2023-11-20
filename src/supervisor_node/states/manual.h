#ifndef SUPERVISOR_NODE_MANUAL_STATE_H
#define SUPERVISOR_NODE_MANUAL_STATE_H

#include "../header/header.h"

/*************************************************** Manual State *****************************************************/
class ManualState : public yasmin::State {
private:
    // parameters
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr manual_state_pub_{};
    string next_state_, manual_command_, general_driver_response_;
    bool lost_liveliness_;

public:
    /// constructor
    ManualState(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub) : yasmin::State({"M>I", "M>A", "M>ES"}),
                                                                            manual_state_pub_(pub) {};
    /// execution
    string execute(shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
        publishing(manual_state_pub_, ManualState::to_string());
        string stdout_message = "Executing state " + ManualState::to_string() + "\n\n"
                                                                                "The vehicle is in manual driving mode, therefore:\n"
                                                                                "- no fault checks are performed;\n"
                                                                                "- all control commands provided by the primary and secondary stack are ignored.";
        string stdout_buffer;
        this->lost_liveliness_ = false;
        if (blackboard->get<string>("previous_state") == ET || blackboard->get<string>("previous_state") == ES)  // optional timer
            this_thread::sleep_for(chrono::milliseconds(END_SLEEP));
        blackboard->set<string>("previous_state", ManualState::to_string());  // memorizing last state

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
            if (this->lost_liveliness_) {  /// LOST LIVELINESS
                blackboard->set<string>("error_cause_ES", timestamp("LOST LIVELINESS", 1));
                return "M>ES";
            } else if (this->next_state_ == I) {  // checking if IDLE state has been selected
                return "M>I";
            } else if (this->next_state_ == A) {  // checking if ACTIVE state has been selected
                return "M>A";
            }
        } while(true);
    }

    /// other methods
    void set_next_state(string str) {
        this->next_state_ = str;
    }

    void set_manual_command(string str) {
        this->manual_command_ = str;
    }

    void set_general_driver_response(string str) {
        this->general_driver_response_ = str;
    }

    void set_lost_liveliness() {
        this->lost_liveliness_ = true;
    }

    string to_string() {
        return M;
    }
};

#endif //SUPERVISOR_NODE_MANUAL_STATE_H