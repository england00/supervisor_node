#include <functional>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#define I "IDLE"
#define M "MANUAL"
#define A "ACTIVE"
#define ET "EMERGENCY TAKEOVER"
#define ES "EMERGENCY STOP"
#define END "END"

#define SLEEP 5
#define CURRENT_STATE_TOPIC "supervisor_node/current_state"
#define STATE_SELECTION_TOPIC "supervisor_node/state_selection"
#define MANUAL_COMMAND_TOPIC "supervisor_node/manual_command"
#define PRIMARY_DRIVING_STACK_TOPIC "supervisor_node/primary_driving_stack"

using namespace std;

/**************************************************** Methods *********************************************************/
atomic_bool stop_thread;

bool input_integer_checker(string &input, int floor, int ceiling) {
    int number;
    std::istringstream iss(input);
    if(iss >> number) {
        if(number >= floor && number <= ceiling) {
            return true;
        } else {
            cerr << "ERROR. Invalid choice selected.\n" << endl;
            return false;
        }
    } else {
        cerr << "ERROR. Input is not an integer.\n" << endl;
        return false;
    }
}


/******************************************** External State Selector Node ********************************************/
class ExternalStateSelectorNode : public rclcpp::Node {
private:
    // parameters
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr current_state_subscription_{};
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publish_state_selection_{};
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publish_primary_driving_stack_command_{};
    string current_state_, state_selection_;

    // different threads for simulation
    thread timer_1_;

    // methods
    void current_state_subscription(const std_msgs::msg::String::SharedPtr msg) {
        this->current_state_ = msg->data;
        system("clear");
        cout << "EXTERNAL STATE SELECTOR NODE:\n" << endl;

        /// ---> polling publisher activation in a different thread
        if (this->current_state_ == A) {
            this->timer_1_ = thread([this]() {  // opening new simulation thread
                polling_publishing(this->publish_primary_driving_stack_command_, "PRIMARY DRIVING STACK");
            });
        }

        // selection cycle
        bool check = false;
        if (this->current_state_ != END) {
            do {
                cout << "SUPERVISOR NODE --> ON\n"
                        "Current state: " << this->current_state_ <<
                        "\n\nType the digit of the option you want to choose:" << endl;

                /// behaviour based on current state
                // handling IDLE state
                if (this->current_state_ == I) {
                    cout << "1. Pass to " + string(END) << endl;
                    cout << "2. Pass to " + string(M) << endl;
                    cin >> this->state_selection_;
                    cin.ignore();  // cleaning input buffer
                    system("clear");
                    cout << "EXTERNAL STATE SELECTOR NODE:\n" << endl;
                    if (input_integer_checker(this->state_selection_, 1, 2))
                        check = true;
                }
                // handling MANUAL state
                else if (this->current_state_ == M) {
                    cout << "1. Pass to " + string(I) << endl;
                    cout << "2. Pass to " + string(A) << endl;
                    cin >> this->state_selection_;
                    cin.ignore();  // cleaning input buffer
                    system("clear");
                    cout << "EXTERNAL STATE SELECTOR NODE:\n" << endl;
                    if (input_integer_checker(this->state_selection_, 1, 2))
                        check = true;
                }
                // handling ACTIVE state
                else if (this->current_state_ == A) {
                    cout << "1. Pass to " + string(M) << endl;
                    cin >> this->state_selection_;
                    cin.ignore();  // cleaning input buffer
                    system("clear");
                    cout << "EXTERNAL STATE SELECTOR NODE:\n" << endl;
                    if (input_integer_checker(this->state_selection_, 1, 1)) {
                        check = true;
                    }

                }
            } while (!check);

            /// publishing selection
            // from IDLE
            if (this->current_state_ == I) {
                if (this->state_selection_ == "1") {
                    this->publishing(this->publish_state_selection_, END);
                } else if (this->state_selection_ == "2") {
                    this->publishing(this->publish_state_selection_, M);
                }
            }
            // from MANUAL
            else if (this->current_state_ == M) {
                if (this->state_selection_ == "1") {
                    this->publishing(this->publish_state_selection_, I);
                } else if (this->state_selection_ == "2") {
                    this->publishing(this->publish_state_selection_, A);
                }
            }
            // from ACTIVE
            else if (this->current_state_ == A) {
                /// ---> joining simulation thread
                stop_thread = true;
                this->timer_1_.join();
                if (this->state_selection_ == "1") {
                    this->publishing(this->publish_state_selection_, M);
                }
            }
        } else {
            cout << "SUPERVISOR NODE --> OFF" << endl;
            exit(EXIT_SUCCESS);
        }
    }

    void publishing(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub, const std::string &msg) {
        auto message = std_msgs::msg::String();
        message.data = msg;
        pub->publish(message);
    }

    static void polling_publishing(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub, const std::string &msg) {
        auto message = std_msgs::msg::String();
        stop_thread = false;
        while (!stop_thread) {
            message.data = msg;
            pub->publish(message);
            this_thread::sleep_for(chrono::milliseconds(SLEEP));  // timer
        }
    }

public:
    // constructor
    ExternalStateSelectorNode() : Node("external_state_selector_node") {
        this->current_state_subscription_ = this->create_subscription<std_msgs::msg::String>(
            CURRENT_STATE_TOPIC,
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
            std::bind(&ExternalStateSelectorNode::current_state_subscription, this, placeholders::_1)
        );
        this->publish_state_selection_ = this->create_publisher<std_msgs::msg::String>(
            STATE_SELECTION_TOPIC,
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable()
        );
        this->publish_primary_driving_stack_command_ = this->create_publisher<std_msgs::msg::String>(
            PRIMARY_DRIVING_STACK_TOPIC,
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable()
        );
    }
};


/******************************************************* Main *********************************************************/
int main(int argc, char * argv[]) {
    // managing what printing on stdout
    cout << "EXTERNAL STATE SELECTOR NODE:\n\nSUPERVISOR NODE --> OFF\n" << endl;

    // C++ idiomatic interface which provides all the ROS client functionality like creating nodes, publisher, and subscribers
    rclcpp::init(argc, argv);  // activation of rclcpp API
    rclcpp::spin(std::make_shared<ExternalStateSelectorNode>()); // node creation and spinning
    rclcpp::shutdown();  // shutdown of rclcpp API
    return 0;
}