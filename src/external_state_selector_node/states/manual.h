#ifndef SUPERVISOR_NODE_MANUAL_H
#define SUPERVISOR_NODE_MANUAL_H

#include "../header/header.h"

/*************************************************** Manual State *****************************************************/
class ManualState : public yasmin::State {
private:
    // parameters
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr manual_state_pub_{};
    string current_state_, state_selection_;

public:
    /// constructor
    ManualState(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub) : yasmin::State({"M>I", "M>A", "M>ES"}),
                                                                            manual_state_pub_(pub) {};
    /// execution
    string execute(shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
        blackboard->set<string>("previous_state", ManualState::to_string());  // memorizing last state

        // state cycle
        do {
            // managing what printing on stdout
            system("clear");
            cout << "EXTERNAL STATE SELECTOR NODE:\n\nSUPERVISOR NODE --> ON\nCurrent state: " << ManualState::to_string() << endl;
            cout << "\nNOTE: if this 'Current state' is not equal to SUPERVISOR NODE's Current state, type one of the following options to reload."
                    "\n\nType the digit of the option you want to choose:" << endl;
            cout << "1. Pass to " + string(I) << endl;
            cout << "2. Pass to " + string(A) << endl;
            cin >> this->state_selection_;  // expecting keyboard input
            cin.ignore();  // cleaning input buffer

            // checking if current state is changed during the state selection
            if (this->current_state_ != ManualState::to_string()) {
                cout << "Updating to effective CURRENT STATE" << endl;
                this_thread::sleep_for(chrono::milliseconds(UPDATE_CURRENT_STATE_SLEEP));
                if (this->current_state_ == ES) {
                    return "M>ES";
                }
            }

            // checking keyboard input correctness
            if (input_integer_checker(this->state_selection_, 1, 2)) {
                if (this->state_selection_ == "1") {
                    publisher(this->manual_state_pub_, I);
                    return "M>I";
                } else if (this->state_selection_ == "2") {
                    publisher(this->manual_state_pub_, A);
                    return "M>A";
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
        return M;
    }
};

#endif //SUPERVISOR_NODE_MANUAL_H