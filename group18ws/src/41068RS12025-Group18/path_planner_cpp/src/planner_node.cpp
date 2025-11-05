#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "path_planner_cpp/srv/plan_to_goal.hpp"

using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
using GoalHandleComputePathToPose = rclcpp_action::ClientGoalHandle<ComputePathToPose>;

class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode() : Node("planner_node"),
                  goal_x_(0.0), goal_y_(0.0), goal_z_(0.0)
  {
    publisher_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

    // Create the Nav2 planner action client
    action_client_ = rclcpp_action::create_client<ComputePathToPose>(
        this, "/compute_path_to_pose");

    // Create the new service to plan to a specific goal
    plan_service_ = this->create_service<path_planner_cpp::srv::PlanToGoal>(
        "plan_to_goal",
        std::bind(&PlannerNode::handle_plan_to_goal, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Keep the old trigger service for backward compatibility (uses 0,0,0)
    trigger_service_ = this->create_service<std_srvs::srv::Trigger>(
        "trigger_path_plan",
        std::bind(&PlannerNode::handle_trigger, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(),
                "PlannerNode ready. Services available:");
    RCLCPP_INFO(this->get_logger(),
                "  - /plan_to_goal (accepts x, y, z coordinates)");
    RCLCPP_INFO(this->get_logger(),
                "  - /trigger_path_plan (plans to origin 0,0,0)");
  }

private:
  rclcpp_action::Client<ComputePathToPose>::SharedPtr action_client_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_service_;
  rclcpp::Service<path_planner_cpp::srv::PlanToGoal>::SharedPtr plan_service_;

  // Goal coordinates
  double goal_x_, goal_y_, goal_z_;

  void handle_plan_to_goal(
      const std::shared_ptr<path_planner_cpp::srv::PlanToGoal::Request> request,
      std::shared_ptr<path_planner_cpp::srv::PlanToGoal::Response> response)
  {
    RCLCPP_INFO(this->get_logger(),
                "Service called: planning to goal (%.2f, %.2f, %.2f)",
                request->x, request->y, request->z);

    if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
      RCLCPP_ERROR(this->get_logger(), "Planner action server not available!");
      response->success = false;
      response->message = "Planner server unavailable.";
      return;
    }

    // Store goal coordinates
    goal_x_ = request->x;
    goal_y_ = request->y;
    goal_z_ = request->z;

    send_goal();

    response->success = true;
    response->message = "Path computation to (" +
                       std::to_string(request->x) + ", " +
                       std::to_string(request->y) + ", " +
                       std::to_string(request->z) + ") started.";
  }

  void handle_trigger(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request; // Unused
    RCLCPP_INFO(this->get_logger(),
                "Trigger service called: computing path to origin (0, 0, 0)...");

    if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
      RCLCPP_ERROR(this->get_logger(), "Planner action server not available!");
      response->success = false;
      response->message = "Planner server unavailable.";
      return;
    }

    // Default goal is origin
    goal_x_ = 0.0;
    goal_y_ = 0.0;
    goal_z_ = 0.0;

    send_goal();

    response->success = true;
    response->message = "Path computation to origin started.";
  }

  void send_goal()
  {
    auto goal_msg = ComputePathToPose::Goal();

    // Set goal pose using stored coordinates (Nav2 uses robot's current position as start)
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = this->now();
    goal.pose.position.x = goal_x_;
    goal.pose.position.y = goal_y_;
    goal.pose.position.z = goal_z_;
    goal.pose.orientation.w = 1.0;
    goal_msg.goal = goal;

    // Send goal asynchronously
    auto send_goal_options =
        rclcpp_action::Client<ComputePathToPose>::SendGoalOptions();
    send_goal_options.result_callback =
        [this](const GoalHandleComputePathToPose::WrappedResult &result) {
          if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(),
                        "Path successfully computed, publishing to /path");
            publisher_->publish(result.result->path);
          } else {
            RCLCPP_ERROR(this->get_logger(), "Path computation failed!");
          }
        };

    action_client_->async_send_goal(goal_msg, send_goal_options);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


