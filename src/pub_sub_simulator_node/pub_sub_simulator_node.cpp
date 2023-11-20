#include "header/header.h"
#include "states/idle.h"
#include "states/manual.h"
#include "states/active.h"
#include "states/emergency_takeover.h"
#include "states/emergency_stop.h"

/********************************************** Pub/Sub Simulator Node ************************************************/
class PubSubSimulatorNode : public simple_node::Node {
private:
    // blackboard
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard_ = std::make_shared<yasmin::blackboard::Blackboard>();

    // publishers and subscribers
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr selected_state_sub_ = nullptr;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr current_state_subscription_ = nullptr;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publish_manual_command_ = nullptr;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publish_common_fault_ = nullptr;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publish_primary_driving_stack_command_ = nullptr;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publish_secondary_driving_stack_command_ = nullptr;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publish_general_driver_response_ = nullptr;

    // states
    std::shared_ptr<IdleState> idleState_ = nullptr;
    std::shared_ptr<ManualState> manualState_ = nullptr;
    std::shared_ptr<ActiveState> activeState_ = nullptr;
    std::shared_ptr<EmergencyTakeoverState> emergencyTakeoverState_ = nullptr;
    std::shared_ptr<EmergencyStopState> emergencyStopState_ = nullptr;

public:
    /// constructor
    PubSubSimulatorNode() : simple_node::Node("pub_sub_simulator_node") {

        // quality of service
        rclcpp::QoS qos_profile(rclcpp::KeepLast(PUB_SUB_QUEUE));  // queue dimension
        qos_profile.reliable();  // type of communication

        /// SELECTED STATE SUBSCRIPTION
        this->selected_state_sub_ = this->create_subscription<std_msgs::msg::String>(STATE_SELECTION_TOPIC,
            qos_profile,
            std::bind(&PubSubSimulatorNode::selected_state_subscription, this, placeholders::_1)
        );
        /// CURRENT STATE SUBSCRIPTION
        qos_profile.transient_local();
        this->current_state_subscription_ = this->create_subscription<std_msgs::msg::String>(CURRENT_STATE_TOPIC,
            qos_profile,
            std::bind(&PubSubSimulatorNode::current_state_subscription, this, placeholders::_1)
        );
        /// MANUAL COMMANDS PUBLISHER
        this->publish_manual_command_ = this->create_publisher<std_msgs::msg::String>(MANUAL_COMMAND_TOPIC,
            qos_profile
        );
        /// COMMON FAULTS PUBLISHER
        this->publish_common_fault_ = this->create_publisher<std_msgs::msg::String>(COMMON_FAULT_TOPIC,
            qos_profile
        );
        /// PRIMARY DRIVING STACK PUBLISHER
        qos_profile.deadline(chrono::milliseconds(DEADLINE_PRIMARY_STACK));
        this->publish_primary_driving_stack_command_ = this->create_publisher<std_msgs::msg::String>(
                PRIMARY_DRIVING_STACK_TOPIC,
                qos_profile
        );
        /// SECONDARY DRIVING STACK PUBLISHER
        qos_profile.deadline(chrono::milliseconds(DEADLINE_SECONDARY_STACK));
        this->publish_secondary_driving_stack_command_ = this->create_publisher<std_msgs::msg::String>(
                SECONDARY_DRIVING_STACK_TOPIC,
                qos_profile
        );
        /// GENERAL DRIVER RESPONSE PUBLISHER
        qos_profile.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC).liveliness_lease_duration(chrono::milliseconds(GENERAL_DRIVER_LIVELINESS));
        this->publish_general_driver_response_ = this->create_publisher<std_msgs::msg::String>(
                GENERAL_DRIVER_TOPIC,
                qos_profile
        );

        // finite state machine creation
        this->idleState_ = std::make_shared<IdleState>();
        this->manualState_ = std::make_shared<ManualState>(this->publish_manual_command_, this->publish_general_driver_response_);
        this->activeState_ = std::make_shared<ActiveState>(this->publish_primary_driving_stack_command_, this->publish_common_fault_, this->publish_general_driver_response_);
        this->emergencyTakeoverState_ = std::make_shared<EmergencyTakeoverState>(this->publish_secondary_driving_stack_command_, this->publish_general_driver_response_);
        this->emergencyStopState_ = std::make_shared<EmergencyStopState>();
        auto fsm = std::make_shared<yasmin::StateMachine>(yasmin::StateMachine({END}));

        // add states
        fsm->add_state(this->idleState_->to_string(), this->idleState_, {
                {"I>M", this->manualState_->to_string()},
                {"I>End", END}
        });
        fsm->add_state(this->manualState_->to_string(), this->manualState_, {
                {"M>I", this->idleState_->to_string()},
                {"M>A", this->activeState_->to_string()},
                {"M>ES", this->emergencyStopState_->to_string()}
        });
        fsm->add_state(this->activeState_->to_string(), this->activeState_, {
                {"A>M", this->manualState_->to_string()},
                {"A>ET", this->emergencyTakeoverState_->to_string()},
                {"A>ES", this->emergencyStopState_->to_string()}
        });
        fsm->add_state(this->emergencyTakeoverState_->to_string(), this->emergencyTakeoverState_, {
                {"ET>M", this->manualState_->to_string()},
                {"ET>A", this->activeState_->to_string()},
                {"ET>ES", this->emergencyStopState_->to_string()}
        });
        fsm->add_state(this->emergencyStopState_->to_string(), this->emergencyStopState_, {
                {"ES>M", this->manualState_->to_string()},
                {"ES>ET", this->emergencyTakeoverState_->to_string()}
        });

        // executing fsm
        fsm->execute(this->blackboard_);

        // exit
        end_execution();
    }

    /// other methods
    // passing SELECTED STATE
    void selected_state_subscription(const std_msgs::msg::String::SharedPtr msg) {
        this->idleState_->set_selected_state(msg->data);
        this->manualState_->set_selected_state(msg->data);
        this->activeState_->set_selected_state(msg->data);
        this->emergencyTakeoverState_->set_selected_state(msg->data);
        this->emergencyStopState_->set_selected_state(msg->data);
    }
    // passing SUPERVISOR NODE CURRENT STATE
    void current_state_subscription(const std_msgs::msg::String::SharedPtr msg) {
        this->idleState_->set_current_state(msg->data);
        this->manualState_->set_current_state(msg->data);
        this->activeState_->set_current_state(msg->data);
        this->emergencyTakeoverState_->set_current_state(msg->data);
        this->emergencyStopState_->set_current_state(msg->data);
    }
};


/******************************************************* Main *********************************************************/
int main(int argc, char *argv[]) {
    // managing what printing on stdout
    cout << "PUB/SUB SIMULATOR NODE:\n\nSUPERVISOR NODE --> OFF\n" << endl;

    // C++ idiomatic interface which provides all the ROS client functionality like creating nodes, publisher, and subscribers
    rclcpp::init(argc, argv);  // activation of rclcpp API
    rclcpp::spin(std::make_shared<PubSubSimulatorNode>());  // node creation and spinning
    rclcpp::shutdown();  // shutdown of rclcpp API
    return 0;
}