#include "header/header.h"
#include "states/idle.h"
#include "states/manual.h"
#include "states/active.h"
#include "states/emergency_takeover.h"
#include "states/emergency_stop.h"

/******************************************** External State Selector Node ********************************************/
class ExternalStateSelectorNode : public simple_node::Node {
private:
    // blackboard
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard_ = std::make_shared<yasmin::blackboard::Blackboard>();

    // publishers and subscribers
    /// CURRENT STATE SUBSCRIPTION
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr current_state_subscription_ = this->create_subscription<std_msgs::msg::String>(
            CURRENT_STATE_TOPIC,
            rclcpp::QoS(rclcpp::KeepLast(PUB_SUB_QUEUE)).reliable().transient_local(),
            std::bind(&ExternalStateSelectorNode::current_state_subscription, this, placeholders::_1)
    );
    /// STATE SELECTION PUBLISHER
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publish_state_selection_ = this->create_publisher<std_msgs::msg::String>(
            STATE_SELECTION_TOPIC,
            rclcpp::QoS(rclcpp::KeepLast(PUB_SUB_QUEUE)).reliable()
    );

    // states
    std::shared_ptr<IdleState> idleState_ = std::make_shared<IdleState>(this->publish_state_selection_);
    std::shared_ptr<ManualState> manualState_ = std::make_shared<ManualState>(this->publish_state_selection_);
    std::shared_ptr<ActiveState> activeState_ = std::make_shared<ActiveState>(this->publish_state_selection_);
    std::shared_ptr<EmergencyTakeoverState> emergencyTakeoverState_ = std::make_shared<EmergencyTakeoverState>(this->publish_state_selection_);
    std::shared_ptr<EmergencyStopState> emergencyStopState_ = std::make_shared<EmergencyStopState>();

public:
    /// constructor
    ExternalStateSelectorNode() : simple_node::Node("external_state_selector_node") {

        // finite state machine creation
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
int main(int argc, char * argv[]) {
    // managing what printing on stdout
    cout << "EXTERNAL STATE SELECTOR NODE:\n\nSUPERVISOR NODE --> OFF\n" << endl;

    // C++ idiomatic interface which provides all the ROS client functionality like creating nodes, publisher, and subscribers
    rclcpp::init(argc, argv);  // activation of rclcpp API
    rclcpp::spin(std::make_shared<ExternalStateSelectorNode>()); // node creation and spinning
    rclcpp::shutdown();  // shutdown of rclcpp API
    return 0;
}