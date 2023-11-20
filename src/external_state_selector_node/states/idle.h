#ifndef SUPERVISOR_NODE_IDLE_H
#define SUPERVISOR_NODE_IDLE_H

#include "../header/header.h"

/**************************************************** Idle State ******************************************************/
class IdleState : public yasmin::State {
private:
    // parameters
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr idle_state_pub_{};
    string current_state_, state_selection_;

public:
    /// constructor
    IdleState(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub) : yasmin::State({"I>M", "I>End"}),
                                                                          idle_state_pub_(pub) {};
    /// execution
    string execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
        blackboard->set<string>("previous_state", IdleState::to_string());  // memorizing last state

        // state cycle
        do {
            // checking if SUPERVISOR NODE is on
            if (!this->current_state_.empty()) {
                // managing what printing on stdout
                system("clear");
                cout << "EXTERNAL STATE SELECTOR NODE:\n\nSUPERVISOR NODE --> ON\nCurrent state: " << IdleState::to_string() << endl;
                cout << "\nNOTE: if this 'Current state' is not equal to SUPERVISOR NODE's Current state, type one of the following options to reload."
                        "\n\nType the digit of the option you want to choose:" << endl;
                cout << "1. Pass to " + string(END) << endl;
                cout << "2. Pass to " + string(M) << endl;
                cin >> this->state_selection_;  // expecting keyboard input
                cin.ignore();  // cleaning input buffer

                // checking keyboard input correctness
                if (input_integer_checker(this->state_selection_, 1, 2)) {
                    if (this->state_selection_ == "1") {
                        publisher(this->idle_state_pub_, END);
                        return "I>End";
                    } else if (this->state_selection_ == "2") {
                        publisher(this->idle_state_pub_, M);
                        return "I>M";
                    }
                } else {
                    // some time to show error message
                    this_thread::sleep_for(chrono::milliseconds(UPDATE_CURRENT_STATE_SLEEP));
                }

                // expecting to receive last current state
                this_thread::sleep_for(chrono::milliseconds(UPDATE_CURRENT_STATE_SLEEP));  // timer
            }
        } while(true);
    }

    /// other methods
    void set_current_state(string str) {
        this->current_state_ = str;
    }

    string to_string() {
        return I;
    }
};

#endif //SUPERVISOR_NODE_IDLE_H