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
#define GENERAL_SLEEP 30  // milliseconds
#define MANUAL_COMMAND_PUBLISHER_SLEEP 1000  // milliseconds
#define ACTIVE_SLEEP 10 // milliseconds
#define PRIMARY_DRIVING_STACK_PUBLISHER_SLEEP 100  // milliseconds
#define SECONDARY_DRIVING_STACK_PUBLISHER_SLEEP 100  // milliseconds
#define GENERAL_DRIVER_RESPONSE_SLEEP 1000 // milliseconds
#define END_SLEEP 500  // milliseconds
#define DEADLINE_PRIMARY_STACK 200  // milliseconds
#define DEADLINE_SECONDARY_STACK 300  // milliseconds
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
int generateRandomInt(int min, int max);
string choose_manual_commands();
string choose_common_fault();
string choose_general_driver_response();
string timestamp(string command);
extern atomic_bool stop_thread;
void polling_publisher(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub, const std::string &state, int delay, int thread_number=0);
void end_execution();

#endif //SUPERVISOR_NODE_HEADER_H