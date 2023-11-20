#ifndef SUPERVISOR_NODE_IDLE_STATE_H
#define SUPERVISOR_NODE_IDLE_STATE_H

#include "../header/supervisor_node_header.h"

/**************************************************** Idle State ******************************************************/
class IdleState : public yasmin::State {
private:
    // parameters
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr idle_state_pub_{};
    string next_state_;

public:
    /// constructor
    IdleState(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub) : yasmin::State({"I>M", "I>End"}),
                                                                          idle_state_pub_(pub) {};
    /// execution
    string execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
        publishing(idle_state_pub_, IdleState::to_string());
        string stdout_message = "Executing state " + IdleState::to_string() + "\n\n"
                                                                              "The node is on and awaits signals from the outside.";
        blackboard->set<string>("previous_state", IdleState::to_string());  // memorizing last state

        // state cycle
        do {
            // managing what printing on stdout
            system("clear");
            cout << "SUPERVISION NODE:\n" << endl;
            cout << stdout_message << endl;
            this_thread::sleep_for(chrono::milliseconds(GENERAL_SLEEP));  // timer

            // managing transitions
            if (this->next_state_ == M) {  // checking if MANUAL state has been selected
                return "I>M";
            } else if (this->next_state_ == END) {  // checking if END state has been selected
                return "I>End";
            }
        } while(true);
    }

    /// other methods
    void set_next_state(string str) {  this->next_state_ = str;  }
    string to_string() {  return I;  }
};

#endif //SUPERVISOR_NODE_IDLE_STATE_H