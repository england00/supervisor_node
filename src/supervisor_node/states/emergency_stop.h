#ifndef SUPERVISOR_NODE_EMERGENCY_STOP_H
#define SUPERVISOR_NODE_EMERGENCY_STOP_H

#include "../header/supervisor_node_header.h"

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
        publishing(emergency_stop_state_pub_, EmergencyStopState::to_string());
        string stdout_message = "Executing state " + EmergencyStopState::to_string() + "\n\n"
                                                                                       "The vehicle is unable to move autonomously:\n"
                                                                                       "- driving commands are ignored and the vehicle is brought to a stop.";
        string stdout_buffer;
        this->previous_state_ = blackboard->get<string>("previous_state");
        blackboard->set<string>("previous_state", EmergencyStopState::to_string());  // memorizing last state
        clock_t begin = clock(), current_time;  // timer start

        // state cycle
        do {
            // publishing current state in each iteration
            publishing(emergency_stop_state_pub_, EmergencyStopState::to_string());

            // managing what printing on stdout
            system("clear");
            cout << "SUPERVISION NODE:\n" << endl;
            cout << stdout_message << endl;  // default output
            cout << blackboard->get<string>("error_cause_ES") << endl;  // emergency type
            stdout_buffer = timestamp("SERIOUS FAULT", 2);
            cout << stdout_buffer << endl;
            this_thread::sleep_for(chrono::milliseconds(GENERAL_SLEEP));  // timer

            // managing transition
            current_time = clock() - begin;
            if (current_time > EMERGENCY_TIMER and this->previous_state_ == M) {   /// return to PREVIOUS STATE
                return "ES>M";  // --> the serious fault is resolved
            } else if (current_time > EMERGENCY_TIMER and (this->previous_state_ == A || this->previous_state_ == ET)) {  /// return to PREVIOUS STATE
                return "ES>ET";  // --> the serious fault is resolved
            }
        } while(true);
    }

    /// other methods
    void set_next_state(string str) {  this->next_state_ = str;  }
    string to_string() {  return ES;  }
};

#endif //SUPERVISOR_NODE_EMERGENCY_STOP_H