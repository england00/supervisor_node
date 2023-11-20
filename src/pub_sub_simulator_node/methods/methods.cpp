#include "../header/header.h"

/***************************************************** Methods ********************************************************/
int generateRandomInt(int min, int max) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(min, max);
    return dis(gen);
}

string choose_manual_commands() {
    int choise = generateRandomInt(1, 5);
    string command;
    if (choise == 1) {  command = "GO STRAIGHT";  }
    else if (choise == 2) {  command = "TURN RIGHT";  }
    else if (choise == 3) {  command = "TURN LEFT";  }
    else if (choise == 4) {  command = "GO BACK RIGHT";  }
    else if (choise == 5) {  command = "STOP";  }
    return command;
}

string choose_common_fault() {
    int choise = generateRandomInt(1, 100);
    string command;
    if (choise == 1) {  command = "ADHESION LOSS";  }
    else if (choise == 5) {  command = "CERTAIN COLLISION SEQUENCE";  }
    else {  command = "";  }
    return command;
}

string choose_general_driver_response() {
    int choise = generateRandomInt(1, 10);
    string command;
    if (choise == 5) {  command = "";  }
    else {  command = "GENERAL DRIVER RESPONSE";  }
    return command;
}

string timestamp(string command) {
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

    return "\nSent message on " + std::string(time_buffer) + ":\n --> " + command;
}

atomic_bool stop_thread;
void polling_publisher(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub, const std::string &state, int delay, int thread_number) {
    auto message = std_msgs::msg::String();
    string commands;
    stop_thread = false;
    while (!stop_thread) {
        // different published messages based on current state
        if (state == M) {  // MANUAL STATE
            if (thread_number == 0) {
                message.data = choose_manual_commands();
                commands = "MANUAL COMMANDS";
            } else if (thread_number == 2) {
                message.data = choose_general_driver_response();
                commands = "GENERAL DRIVER RESPONSE";
            }
        } else if (state == A) {  // ACTIVE STATE
            if (thread_number == 0) {
                message.data = "PRIMARY DRIVING STACK";
                commands = "PRIMARY DRIVING STACK";
            } else if (thread_number == 1) {
                message.data = choose_common_fault();
                commands = "COMMON FAULT";
            } else if (thread_number == 2) {
                message.data = choose_general_driver_response();
                commands = "GENERAL DRIVER RESPONSE";
            }
        } else if (state == ET) {  // EMERGENCY TAKEOVER STATE
            if (thread_number == 0) {
                message.data = "SECONDARY DRIVING STACK";
                commands = "SECONDARY DRIVING STACK";
            } else if (thread_number == 2) {
                message.data = choose_general_driver_response();
                commands = "GENERAL DRIVER RESPONSE";
            }
        }
        if (thread_number == 0) {
            system("clear");
            cout << "PUB/SUB SIMULATOR NODE:\n\n"
                    "SUPERVISOR NODE --> ON\n"
                    "Current state: " <<
                 state << "\n\nPublishing " << commands << " and GENERAL DRIVER RESPONSE:" << endl;
        }
        if (message.data != "") {
            pub->publish(message);  // publishing
            if (thread_number != 1)
                cout << timestamp(message.data) << endl;
        }
        this_thread::sleep_for(chrono::milliseconds(delay));  // timer
    }
}

void end_execution() {
    system("clear");
    cout << "PUB/SUB SIMULATOR NODE:\n\nSUPERVISOR NODE --> OFF" << endl;
    this_thread::sleep_for(chrono::milliseconds(END_SLEEP));  // timer
    exit(EXIT_SUCCESS);
}