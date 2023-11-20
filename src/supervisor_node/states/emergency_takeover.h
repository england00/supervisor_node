#ifndef SUPERVISOR_NODE_EMERGENCY_TAKEOVER_H
#define SUPERVISOR_NODE_EMERGENCY_TAKEOVER_H

#include "../header/supervisor_node_header.h"

/********************************************* Emergency Takeover State ***********************************************/
class EmergencyTakeoverState : public yasmin::State {
private:
    // parameters
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr emergency_takeover_state_pub_{};
    string next_state_, secondary_driving_stack_command_, general_driver_response_;
    bool deadline_missed_, lost_liveliness_;

public:
    /// constructor
    EmergencyTakeoverState(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub) : yasmin::State({"ET>M", "ET>A", "ET>ES"}),
                                                                                       emergency_takeover_state_pub_(pub) {};
    /// execution
    string execute(shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
        publishing(emergency_takeover_state_pub_, EmergencyTakeoverState::to_string());
        string stdout_message = "Executing state " + EmergencyTakeoverState::to_string() + "\n\n"
                                                                                           "The vehicle is in a state of risk:\n"
                                                                                           "- control is entrusted to the secondary driving stack.";
        string stdout_buffer;
        this->deadline_missed_ = false;
        this->lost_liveliness_ = false;
        if (blackboard->get<string>("previous_state") == ES)  // optional timer
            this_thread::sleep_for(chrono::milliseconds(END_SLEEP));
        blackboard->set<string>("previous_state", EmergencyTakeoverState::to_string());  // memorizing last state
        clock_t begin = clock(), current_time;  // timer start

        // state cycle
        do {
            // publishing current state in each iteration
            publishing(emergency_takeover_state_pub_, EmergencyTakeoverState::to_string());

            // managing what printing on stdout
            system("clear");
            cout << "SUPERVISION NODE:\n" << endl;
            cout << stdout_message << endl;  // default output
            cout << blackboard->get<string>("error_cause_ET") << endl;  // emergency type
            stdout_buffer = timestamp("COMMON FAULT", 2);
            if (!this->secondary_driving_stack_command_.empty()) {  // adding the last command received
                stdout_buffer += "\n" + timestamp(this->secondary_driving_stack_command_);
            }
            cout << stdout_buffer << endl;
            this->secondary_driving_stack_command_ = "";  // cleaning latest command buffer
            this_thread::sleep_for(chrono::milliseconds(GENERAL_SLEEP));  // timer

            // managing transition
            if (this->deadline_missed_) {  /// DEADLINE MISSED
                blackboard->set<string>("error_cause_ES", timestamp("DEADLINE MISSED", 1));
                return "ET>ES";
            }
            if (this->next_state_ == M) {  // checking if MANUAL state has been selected
                return "ET>M";
            }
            current_time = clock() - begin;
            if (current_time > EMERGENCY_TIMER) {  /// return to PREVIOUS STATE
                return "ET>A";  // --> the common fault is resolved
            }
        } while(true);
    }

    /// other methods
    void set_next_state(string str) {  this->next_state_ = str;  }
    void set_secondary_driving_stack_command(string str) {  this->secondary_driving_stack_command_ = str;  }
    void set_deadline_missed() {  this->deadline_missed_ = true;  }
    void set_general_driver_response(string str) {  this->general_driver_response_ = str;  }
    void set_lost_liveliness() {  this->lost_liveliness_ = true;  }
    string to_string() {  return ET;  }
};

#endif //SUPERVISOR_NODE_EMERGENCY_TAKEOVER_H