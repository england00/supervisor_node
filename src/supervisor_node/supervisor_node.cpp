#include "header/header.h"
#include "states/idle.h"
#include "states/manual.h"
#include "states/active.h"
#include "states/emergency_takeover.h"
#include "states/emergency_stop.h"

/************************************************** Supervisor Node ***************************************************/
class SupervisorNode : public simple_node::Node {
private:
    rclcpp::QOSDeadlineRequestedInfo missed_deadline_event_;
    rclcpp::QOSLivelinessChangedInfo lost_liveliness_event_;
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard_ = std::make_shared<yasmin::blackboard::Blackboard>();

    // publishers and subscribers
    /// CURRENT STATE PUBLISHER
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr current_state_pub_ = this->create_publisher<std_msgs::msg::String>(
        CURRENT_STATE_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(PUB_SUB_QUEUE)).reliable().transient_local()
    );
    /// SELECTED STATE SUBSCRIPTION
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr selected_state_sub_ = this->create_subscription<std_msgs::msg::String>(
        STATE_SELECTION_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(PUB_SUB_QUEUE)).reliable(),
        std::bind(&SupervisorNode::selected_state_subscription, this, placeholders::_1)
    );
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr manual_command_sub_ = nullptr;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr common_fault_sub_ = nullptr;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr primary_driving_stack_sub_ = nullptr;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr secondary_driving_stack_sub_ = nullptr;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr general_driver_response_sub_ = nullptr;
    std::unique_ptr<yasmin_viewer::YasminViewerPub> yasmin_pub_{};

    // states
    std::shared_ptr<IdleState> idleState_ = std::make_shared<IdleState>(this->current_state_pub_);
    std::shared_ptr<ManualState> manualState_ = std::make_shared<ManualState>(this->current_state_pub_);
    std::shared_ptr<ActiveState> activeState_ = std::make_shared<ActiveState>(this->current_state_pub_);
    std::shared_ptr<EmergencyTakeoverState> emergencyTakeoverState_ = std::make_shared<EmergencyTakeoverState>(this->current_state_pub_);
    std::shared_ptr<EmergencyStopState> emergencyStopState_ = std::make_shared<EmergencyStopState>(this->current_state_pub_);

public:
    /// constructor
    SupervisorNode() : simple_node::Node("supervisor_node") {

        // quality of service
        rclcpp::QoS qos_profile(rclcpp::KeepLast(PUB_SUB_QUEUE));  // queue dimension
        qos_profile.reliable();  // type of communication

        /// MANUAL COMMANDS SUBSCRIPTION
        this->manual_command_sub_ = this->create_subscription<std_msgs::msg::String>(MANUAL_COMMAND_TOPIC, qos_profile,
            std::bind(&SupervisorNode::manual_command_subscription, this, placeholders::_1)
        );
        /// COMMON FAULTS SUBSCRIPTION
        this->common_fault_sub_ = this->create_subscription<std_msgs::msg::String>(COMMON_FAULT_TOPIC, qos_profile,
            std::bind(&SupervisorNode::common_fault_subscription, this, placeholders::_1)
        );
        /// PRIMARY DRIVING STACK SUBSCRIPTION
        qos_profile.deadline(chrono::milliseconds(DEADLINE_PRIMARY_STACK));  // imposing deadline
        rclcpp::SubscriptionOptions subscription_options_primary_driving_stack;
        subscription_options_primary_driving_stack.event_callbacks.deadline_callback = [this](rclcpp::QOSDeadlineRequestedInfo &event) -> void {
            if (this->blackboard_->get<string>("previous_state") == A)  // only if SUPERVISOR NODE is in ACTIVE state
                this->activeState_->set_deadline_missed();  // managing MISSED DEADLINE
            this->missed_deadline_event_ = event;
        };
        this->primary_driving_stack_sub_ = this->create_subscription<std_msgs::msg::String>(PRIMARY_DRIVING_STACK_TOPIC,
            qos_profile,
            std::bind(&SupervisorNode::primary_driving_stack_subscription, this, placeholders::_1),
            subscription_options_primary_driving_stack
        );
        /// SECONDARY DRIVING STACK SUBSCRIPTION
        qos_profile.deadline(chrono::milliseconds(DEADLINE_SECONDARY_STACK));
        rclcpp::SubscriptionOptions subscription_options_secondary_driving_stack;
        subscription_options_secondary_driving_stack.event_callbacks.deadline_callback = [this](rclcpp::QOSDeadlineRequestedInfo &event) -> void {
            if (this->blackboard_->get<string>("previous_state") == ET)  // only if SUPERVISOR NODE is in EMERGENCY TAKEOVER state
                this->emergencyTakeoverState_->set_deadline_missed();  // managing MISSED DEADLINE
            this->missed_deadline_event_ = event;
        };
        this->secondary_driving_stack_sub_ = this->create_subscription<std_msgs::msg::String>(SECONDARY_DRIVING_STACK_TOPIC,
            qos_profile,
            std::bind(&SupervisorNode::secondary_driving_stack_subscription, this, placeholders::_1),
            subscription_options_secondary_driving_stack
        );
        /// GENERAL DRIVER RESPONSE SUBSCRIPTION
        qos_profile.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC).liveliness_lease_duration(chrono::milliseconds(GENERAL_DRIVER_LIVELINESS));
        rclcpp::SubscriptionOptions subscription_options_general_driver_response;
        subscription_options_general_driver_response.event_callbacks.liveliness_callback = [this](rclcpp::QOSLivelinessChangedInfo &event) -> void {
            if (this->blackboard_->get<string>("previous_state") == M) { // only if SUPERVISOR NODE is in MANUAL state
                this->manualState_->set_lost_liveliness();  // managing LOST LIVELINESS
            } else if (this->blackboard_->get<string>("previous_state") == A) { // only if SUPERVISOR NODE is in ACTIVE state
                this->activeState_->set_lost_liveliness();  // managing LOST LIVELINESS
            } else if (this->blackboard_->get<string>("previous_state") == ET) { // only if SUPERVISOR NODE is in EMERGENCY TAKEOVER state
                this->emergencyTakeoverState_->set_lost_liveliness();  // managing LOST LIVELINESS
            }
            this->lost_liveliness_event_ = event;
        };
        this->general_driver_response_sub_ = this->create_subscription<std_msgs::msg::String>(GENERAL_DRIVER_TOPIC,
            qos_profile,
            std::bind(&SupervisorNode::secondary_driving_stack_subscription, this, placeholders::_1),
            subscription_options_general_driver_response
        );

        // finite state machine creation
        auto fsm = std::make_shared<yasmin::StateMachine>(yasmin::StateMachine({END}));

        // publishing and subscribing for states information
        this->yasmin_pub_ = std::make_unique<yasmin_viewer::
        YasminViewerPub>(yasmin_viewer::YasminViewerPub(this, "SUPERVISOR_NODE", fsm));

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
        string outcome = fsm->execute(this->blackboard_);

        // exit
        publishing(this->current_state_pub_, END);
        end_execution(outcome);
    }

    /// other methods
    // passing SELECTED STATE
    void selected_state_subscription(const std_msgs::msg::String::SharedPtr msg) {
        this->idleState_->set_next_state(msg->data);
        this->manualState_->set_next_state(msg->data);
        this->activeState_->set_next_state(msg->data);
        this->emergencyTakeoverState_->set_next_state(msg->data);
        this->emergencyStopState_->set_next_state(msg->data);
    }
    // passing MANUAL COMMANDS
    void manual_command_subscription(const std_msgs::msg::String::SharedPtr msg) {
        this->manualState_->set_manual_command(msg->data);
    }
    // passing PRIMARY DRIVING STACK
    void primary_driving_stack_subscription(const std_msgs::msg::String::SharedPtr msg) {
        this->activeState_->set_primary_driving_stack_command(msg->data);
    }
    // passing SECONDARY DRIVING STACK
    void secondary_driving_stack_subscription(const std_msgs::msg::String::SharedPtr msg) {
        this->emergencyTakeoverState_->set_secondary_driving_stack_command(msg->data);
    }
    // passing COMMON FAULTS
    void common_fault_subscription(const std_msgs::msg::String::SharedPtr msg) {
        this->activeState_->set_common_fault(msg->data);
    }
    // passing GENERAL DRIVER RESPONSES
    void general_driver_response_subscription(const std_msgs::msg::String::SharedPtr msg) {
        this->manualState_->set_general_driver_response(msg->data);
        this->activeState_->set_general_driver_response(msg->data);
        this->emergencyTakeoverState_->set_general_driver_response(msg->data);
    }
};


/******************************************************* Main *********************************************************/
int main(int argc, char *argv[]) {
    // managing what printing on stdout
    cout << "SUPERVISION NODE:\n" << endl;

    // C++ idiomatic interface which provides all the ROS client functionality like creating nodes, publisher, and subscribers
    rclcpp::init(argc, argv);  // activation of rclcpp API
    rclcpp::spin(std::make_shared<SupervisorNode>());  // node creation and spinning
    rclcpp::shutdown();  // shutdown of rclcpp API
    return 0;
}