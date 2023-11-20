#ifndef SUPERVISOR_NODE_ACTIVE_H
#define SUPERVISOR_NODE_ACTIVE_H

#include "../header/header.h"

/*************************************************** Active State *****************************************************/
class ActiveState : public yasmin::State {
private:
    // parameters
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_state_pub_{};
    string current_state_, state_selection_;

public:
    /// constructor
    ActiveState(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub) : yasmin::State({"A>M", "A>ET", "A>ES"}),
                                                                            active_state_pub_(pub) {};
    /// execution
    string execute(shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
        blackboard->set<string>("previous_state", ActiveState::to_string());  // memorizing last state

        // state cycle
        do {
            // managing what printing on stdout
            system("clear");
            cout << "EXTERNAL STATE SELECTOR NODE:\n\nSUPERVISOR NODE --> ON\nCurrent state: " << ActiveState::to_string() << endl;
            cout << "\nNOTE: if this 'Current state' is not equal to SUPERVISOR NODE's Current state, type one of the following options to reload."
                    "\n\nType the digit of the option you want to choose:" << endl;
            cout << "1. Pass to " + string(M) << endl;
            cin >> this->state_selection_;  // expecting keyboard input
            cin.ignore();  // cleaning input buffer

            // checking if current state is changed during the state selection
            if (this->current_state_ != ActiveState::to_string()) {
                cout << "Updating to effective CURRENT STATE" << endl;
                this_thread::sleep_for(chrono::milliseconds(UPDATE_CURRENT_STATE_SLEEP));
                if (this->current_state_ == ET) {
                    return "A>ET";
                } else if (this->current_state_ == ES) {
                    return "A>ES";
                }
            }

            // checking keyboard input correctness
            if (input_integer_checker(this->state_selection_, 1, 1)) {
                if (this->state_selection_ == "1") {
                    publisher(this->active_state_pub_, M);
                    return "A>M";
                }
            } else {
                // some time to show error message
                this_thread::sleep_for(chrono::milliseconds(UPDATE_CURRENT_STATE_SLEEP));
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
        return A;
    }
};

#endif //SUPERVISOR_NODE_ACTIVE_H