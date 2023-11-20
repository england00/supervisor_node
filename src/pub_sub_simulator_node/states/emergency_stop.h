#ifndef SUPERVISOR_NODE_EMERGENCY_STOP_H
#define SUPERVISOR_NODE_EMERGENCY_STOP_H

#include "../header/header.h"

/*********************************************** Emergency Stop State *************************************************/
class EmergencyStopState : public yasmin::State {
private:
    // parameters
    string selected_state_, current_state_;

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
            cout << "PUB/SUB SIMULATOR NODE:\n\n"
                    "SUPERVISOR NODE --> ON\n"
                    "Current state: " <<
                 this->current_state_ << "\n\nNo publishing." << endl;
            this_thread::sleep_for(chrono::milliseconds(ACTIVE_SLEEP));  // timer

            // managing transitions
            if (this->current_state_ == M) {  // checking if SUPERVISOR NODE has switched to MANUAL state
                return "ES>M";
            } else if (this->current_state_ == ET) {  // checking if SUPERVISOR NODE has switched to EMERGENCY TAKEOVER state
                return "ES>ET";
            }
        } while(this->current_state_ != END);

        // emergency exit
        end_execution();
        return "#";
    }

    /// other methods
    void set_selected_state(string str) {
        this->selected_state_ = str;
    }

    void set_current_state(string str) {
        this->current_state_ = str;
    }

    string to_string() {
        return ES;
    }
};

#endif //SUPERVISOR_NODE_EMERGENCY_STOP_H