#include "../header/supervisor_node_header.h"

/***************************************************** Methods ********************************************************/
void publishing(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub, const std::string &msg) {
    auto message = std_msgs::msg::String();
    message.data = msg;
    pub->publish(message);
}

string timestamp(string command, int print_type) {
    // date and time
    char time_buffer[50];  // buffer for date and current time
    auto now = chrono::system_clock::now();  // current date
    auto time = chrono::system_clock::to_time_t(now);  // current time
    strftime(time_buffer, sizeof(time_buffer), "%Y-%m-%d %H:%M:%S", localtime(&time));

    // milliseconds
    char millis_buffer[10]; // buffer for milliseconds
    auto millis = chrono::duration_cast<chrono::milliseconds>(now.time_since_epoch()) % 1000;
    snprintf(millis_buffer, sizeof(millis_buffer), ".%03d", static_cast<int>(millis.count()));

    // concatenate the two buffers
    strcat(time_buffer, millis_buffer);

    if (print_type == 0) {
        return "\nReceived message on " + std::string(time_buffer) + ":\n --> " + command;
    } else if (print_type == 1) {
        return "\nEmergency on " + std::string(time_buffer) + ":\n --> " + command;
    } else {
        return "\nResolving " + command + " on " + std::string(time_buffer);
    }
}

void end_execution(string outcome) {
    system("clear");
    cout << "SUPERVISION NODE:\n\n" << outcome << " reached" << endl;
    this_thread::sleep_for(chrono::milliseconds(END_SLEEP));  // timer
    exit(EXIT_SUCCESS);
}