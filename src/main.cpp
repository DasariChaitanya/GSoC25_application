#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include "ros2_example/action/bool_action.hpp"

class ActionServerNode : public rclcpp::Node {
public:
    using BoolAction = ros2_example::action::BoolAction;
    using GoalHandle = rclcpp_action::ServerGoalHandle<BoolAction>;

    ActionServerNode() : Node("action_server_node") {
        action_server_ = rclcpp_action::create_server<BoolAction>(
            this,
            "bool_action",
            std::bind(&ActionServerNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ActionServerNode::handle_cancel, this, std::placeholders::_1),
            std::bind(&ActionServerNode::handle_accepted, this, std::placeholders::_1)
        );

        publisher_ = this->create_publisher<std_msgs::msg::Bool>("bool_topic", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(10),
            [this]() {
                auto message = std_msgs::msg::Bool();
                message.data = true; // Example data
                publisher_->publish(message);
            }
        );
    }

private:
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const BoolAction::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received goal request");
        (void)uuid;
        (void)goal;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
        std::thread([this, goal_handle]() {
            auto result = std::make_shared<BoolAction::Result>();
            rclcpp::Rate rate(1);
            for (int i = 0; i < 5; ++i) {
                if (goal_handle->is_canceling()) {
                    result->result = false;
                    goal_handle->canceled(result);
                    RCLCPP_INFO(this->get_logger(), "Goal canceled");
                    return;
                }
                auto feedback = std::make_shared<BoolAction::Feedback>();
                feedback->feedback_string = "Processing...";
                feedback->feedback_int = i;
                goal_handle->publish_feedback(feedback);
                rate.sleep();
            }
            result->result = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }).detach();
    }

    rclcpp_action::Server<BoolAction>::SharedPtr action_server_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ActionServerNode>());
    rclcpp::shutdown();
    return 0;
}