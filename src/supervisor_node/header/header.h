#ifndef SUPERVISOR_NODE_SUPERVISOR_NODE_HEADER_H
#define SUPERVISOR_NODE_SUPERVISOR_NODE_HEADER_H

#include <chrono>
#include <ctime>
#include <iostream>
#include <string>
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
#define GENERAL_SLEEP 100  // milliseconds
#define END_SLEEP 500  // milliseconds
#define EMERGENCY_TIMER 55000
#define DEADLINE_PRIMARY_STACK 300  // milliseconds
#define DEADLINE_SECONDARY_STACK 500  // milliseconds
#define GENERAL_DRIVER_LIVELINESS 2000 // milliseconds

#define PUB_SUB_QUEUE 1
#define CURRENT_STATE_TOPIC "supervisor_node/current_state"
#define STATE_SELECTION_TOPIC "supervisor_node/state_selection"
#define MANUAL_COMMAND_TOPIC "supervisor_node/manual_command"
#define PRIMARY_DRIVING_STACK_TOPIC "supervisor_node/primary_driving_stack"
#define SECONDARY_DRIVING_STACK_TOPIC "supervisor_node/secondary_driving_stack"
#define COMMON_FAULT_TOPIC "supervisor_node/common_fault"
#define GENERAL_DRIVER_TOPIC "supervisor_node/general_sensor_or_actuator_driver_response"

using namespace std;

/***************************************************** Methods ********************************************************/
void publishing(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub, const std::string &msg);
string timestamp(string command, int print_type = 0);
void end_execution(string outcome);

#endif //SUPERVISOR_NODE_SUPERVISOR_NODE_HEADER_H