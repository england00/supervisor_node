#ifndef SUPERVISOR_NODE_HEADER_H
#define SUPERVISOR_NODE_HEADER_H

#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <random>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "simple_node/node.hpp"
#include "yasmin/state.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"

#define I "IDLE"
#define M "MANUAL"
#define A "ACTIVE"
#define ET "EMERGENCY TAKEOVER"
#define ES "EMERGENCY STOP"
#define END "END"

/// All these times are chosen for the correctness of the simulation
#define INPUT_ERROR_SLEEP 200 // milliseconds
#define UPDATE_CURRENT_STATE_SLEEP 100  // milliseconds
#define END_SLEEP 500  // milliseconds

#define PUB_SUB_QUEUE 1
#define CURRENT_STATE_TOPIC "supervisor_node/current_state"
#define STATE_SELECTION_TOPIC "supervisor_node/state_selection"

using namespace std;

/**************************************************** Methods *********************************************************/
bool input_integer_checker(string &input, int floor, int ceiling);
void publisher(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub, const std::string &msg);
void end_execution();

#endif //SUPERVISOR_NODE_HEADER_H