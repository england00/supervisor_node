#ifndef SUPERVISOR_NODE_IDLE_H
#define SUPERVISOR_NODE_IDLE_H

#include "../header/header.h"

/**************************************************** Idle State ******************************************************/
class IdleState : public yasmin::State {
private:
    // parameters
    string selected_state_, current_state_;

public:
    /// constructor
    IdleState() : yasmin::State({"I>M", "I>End"}) {};

    /// execution
    string execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
        blackboard->set<string>("previous_state", IdleState::to_string());  // memorizing last state

        // state cycle
        do {
            // managing what printing on stdout
            if (!this->current_state_.empty()) {
                system("clear");
                cout << "PUB/SUB SIMULATOR NODE:\n\n"
                        "SUPERVISOR NODE --> ON\n"
                        "Current state: " <<
                     this->current_state_ << "\n\n\"The node is on and awaits signals from the outside." << endl;
            }
            this_thread::sleep_for(chrono::milliseconds(GENERAL_SLEEP));  // timer

            // managing transitions
            if (this->selected_state_ == M) {  // checking if SUPERVISOR NODE has switched to MANUAL state
                return "I>M";
            } else if (this->selected_state_ == END) {  // checking if SUPERVISOR NODE has switched to END state
                return "I>End";
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
        return I;
    }
};

#endif //SUPERVISOR_NODE_IDLE_H