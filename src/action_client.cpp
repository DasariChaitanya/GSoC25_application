#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include "ros2_example/action/bool_action.hpp"

class ActionClientNode : public rclcpp::Node {
public:
    using BoolAction = ros2_example::action::BoolAction;

    ActionClientNode() : Node("action_client_node") {
        action_client_ = rclcpp_action::create_client<BoolAction>(this, "bool_action");

        subscription_ = this->create_subscription<std_msgs::msg::Bool>(
            "bool_topic",
            10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (msg->data) {
                    send_goal();
                }
            }
        );
    }

private:
    void send_goal() {
        if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return;
        }

        auto goal_msg = BoolAction::Goal();
        goal_msg.request = true;
        auto send_goal_options = rclcpp_action::Client<BoolAction>::SendGoalOptions();

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        send_goal_options.goal_response_callback = std::bind([](
            std::shared_ptr<rclcpp_action::ClientGoalHandle<BoolAction>> goal_handle) {
            if (!goal_handle) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal was rejected by server");
            } else {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal accepted by server, waiting for result");
            }
        }, std::placeholders::_1);
        send_goal_options.result_callback = std::bind([](
            const rclcpp_action::ClientGoalHandle<BoolAction>::WrappedResult & result) {
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal succeeded");
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal aborted");
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Goal canceled");
                    break;
                default:
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unknown result code");
                    break;
            }
        }, std::placeholders::_1);
        send_goal_options.feedback_callback = std::bind([](
            rclcpp_action::ClientGoalHandle<BoolAction>::SharedPtr,
            const std::shared_ptr<const BoolAction::Feedback> feedback) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received feedback: %s", feedback->feedback_string.c_str());
        }, std::placeholders::_1, std::placeholders::_2);
        auto goal_handle_future = action_client_->async_send_goal(goal_msg, send_goal_options);
    }

   

    rclcpp_action::Client<BoolAction>::SharedPtr action_client_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ActionClientNode>());
    rclcpp::shutdown();
    return 0;
}