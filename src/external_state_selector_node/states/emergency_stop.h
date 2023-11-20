#ifndef SUPERVISOR_NODE_EMERGENCY_STOP_H
#define SUPERVISOR_NODE_EMERGENCY_STOP_H

#include "../header/header.h"

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
    void set_current_state(string str) {
        this->current_state_ = str;
    }

    string to_string() {
        return ES;
    }
};

#endif //SUPERVISOR_NODE_EMERGENCY_STOP_H