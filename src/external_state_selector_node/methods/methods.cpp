#include "../header/header.h"

/**************************************************** Methods *********************************************************/
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

void publisher(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub, const std::string &msg) {
    auto message = std_msgs::msg::String();
    message.data = msg;
    pub->publish(message);
}

void end_execution() {
    system("clear");
    cout << "EXTERNAL STATE SELECTOR NODE:\n\nSUPERVISOR NODE --> OFF" << endl;
    this_thread::sleep_for(chrono::milliseconds(END_SLEEP));  // timer
    exit(EXIT_SUCCESS);
}