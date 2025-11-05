/**
 * Forest Mission Orchestrator Node
 *
 * Integrates all system components for autonomous forest inventory:
 * - Autonomous navigation through tree rows
 * - LIDAR tree detection and measurement
 * - Mission state management
 * - UI integration
 * - Return to home base
 *
 * Meets requirements: R1, R2, R3, R6, R10
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <vector>
#include <cmath>
#include <string>
#include <memory>

enum class MissionState {
    IDLE,
    TAKING_OFF,
    FLYING_TO_START,
    SCANNING_ROW,
    MOVING_TO_NEXT_ROW,
    RETURNING_HOME,
    LANDING,
    COMPLETED,
    EMERGENCY_STOP
};

class ForestMissionOrchestrator : public rclcpp::Node
{
public:
    ForestMissionOrchestrator()
    : Node("forest_mission_orchestrator"),
      state_(MissionState::IDLE),
      current_row_(0),
      current_waypoint_(0),
      target_altitude_(0.5),
      home_x_(-2.0),
      home_y_(-12.0),
      home_z_(0.5),
      current_x_(0.0),
      current_y_(0.0),
      current_z_(0.0),
      tree_detection_count_(0)
    {
        RCLCPP_INFO(get_logger(), "Forest Mission Orchestrator initializing...");

        // Define plantation layout - 3 rows of 6 trees
        // Row 1 (x=-4): y = -10, -6, -2, 2, 6, 10
        // Row 2 (x=0):  y = -10, -6, -2, 2, 6, 10
        // Row 3 (x=4):  y = -10, -6, -2, 2, 6, 10

        // Navigation waypoints for row scanning
        // Strategy: Start at beginning of row, fly to end, move to next row

        // Row 1: Start at (-4, -12), fly to (-4, 12)
        row_waypoints_.push_back({
            {-4.0, -12.0, 0.5},  // Start of row 1
            {-4.0, 12.0, 0.5}    // End of row 1
        });

        // Row 2: Move to (0, 12), fly to (0, -12)
        row_waypoints_.push_back({
            {0.0, 12.0, 0.5},    // Start of row 2
            {0.0, -12.0, 0.5}    // End of row 2 (reverse direction)
        });

        // Row 3: Move to (4, -12), fly to (4, 12)
        row_waypoints_.push_back({
            {4.0, -12.0, 0.5},   // Start of row 3
            {4.0, 12.0, 0.5}     // End of row 3
        });

        // Publishers
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        goal_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
        status_pub_ = create_publisher<std_msgs::msg::String>("/drone/status", 10);

        // Subscribers
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odometry", 10,
            std::bind(&ForestMissionOrchestrator::odom_callback, this, std::placeholders::_1));

        start_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/drone/cmd/start", 10,
            std::bind(&ForestMissionOrchestrator::start_callback, this, std::placeholders::_1));

        stop_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/drone/cmd/stop", 10,
            std::bind(&ForestMissionOrchestrator::stop_callback, this, std::placeholders::_1));

        return_home_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/drone/cmd/return_home", 10,
            std::bind(&ForestMissionOrchestrator::return_home_callback, this, std::placeholders::_1));

        height_sub_ = create_subscription<std_msgs::msg::Float32>(
            "/drone/cmd/height", 10,
            std::bind(&ForestMissionOrchestrator::height_callback, this, std::placeholders::_1));

        tree_widths_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
            "/known_tree_widths", 10,
            std::bind(&ForestMissionOrchestrator::tree_widths_callback, this, std::placeholders::_1));

        // Nav2 action client
        nav_to_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            this, "navigate_to_pose");

        // Control loop timer (10 Hz)
        control_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ForestMissionOrchestrator::control_loop, this));

        // Status publishing timer (1 Hz)
        status_timer_ = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&ForestMissionOrchestrator::publish_status, this));

        RCLCPP_INFO(get_logger(), "Forest Mission Orchestrator ready. Waiting for start command.");
        publish_status();
    }

private:
    // State
    MissionState state_;
    int current_row_;
    int current_waypoint_;
    double target_altitude_;
    double home_x_, home_y_, home_z_;
    double current_x_, current_y_, current_z_;
    int tree_detection_count_;

    // Waypoints: 3 rows, each with start and end points
    std::vector<std::vector<std::array<double, 3>>> row_waypoints_;

    // ROS
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr return_home_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr height_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr tree_widths_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_to_pose_client_;
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr nav_goal_handle_;

    // Callbacks
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        current_z_ = msg->pose.pose.position.z;
    }

    void start_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data && state_ == MissionState::IDLE) {
            RCLCPP_INFO(get_logger(), "Starting autonomous forest inventory mission!");
            state_ = MissionState::TAKING_OFF;
            current_row_ = 0;
            current_waypoint_ = 0;
            tree_detection_count_ = 0;
        }
    }

    void stop_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data) {
            RCLCPP_WARN(get_logger(), "Emergency stop commanded!");
            state_ = MissionState::EMERGENCY_STOP;
            stop_drone();
        }
    }

    void return_home_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data) {
            RCLCPP_INFO(get_logger(), "Return to home commanded");
            state_ = MissionState::RETURNING_HOME;
        }
    }

    void height_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        target_altitude_ = msg->data;
        RCLCPP_INFO(get_logger(), "Target altitude updated to %.2f m", target_altitude_);
    }

    void tree_widths_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
    {
        // Count detected trees (each tree has 3 values: width, x, y)
        tree_detection_count_ = msg->data.size() / 3;
    }

    void stop_drone()
    {
        auto cmd = geometry_msgs::msg::Twist();
        cmd.linear.x = cmd.linear.y = cmd.linear.z = 0.0;
        cmd.angular.x = cmd.angular.y = cmd.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd);
    }

    void control_loop()
    {
        switch (state_) {
            case MissionState::IDLE:
                // Do nothing, waiting for start command
                break;

            case MissionState::TAKING_OFF:
                handle_takeoff();
                break;

            case MissionState::FLYING_TO_START:
                handle_fly_to_start();
                break;

            case MissionState::SCANNING_ROW:
                handle_scanning_row();
                break;

            case MissionState::MOVING_TO_NEXT_ROW:
                handle_moving_to_next_row();
                break;

            case MissionState::RETURNING_HOME:
                handle_returning_home();
                break;

            case MissionState::LANDING:
                handle_landing();
                break;

            case MissionState::COMPLETED:
                // Mission complete, do nothing
                break;

            case MissionState::EMERGENCY_STOP:
                stop_drone();
                break;
        }
    }

    void handle_takeoff()
    {
        // Simple takeoff: apply upward velocity until target altitude reached
        if (current_z_ < target_altitude_ - 0.1) {
            auto cmd = geometry_msgs::msg::Twist();
            cmd.linear.z = 0.3;  // Gentle ascent
            cmd_vel_pub_->publish(cmd);
        } else {
            RCLCPP_INFO(get_logger(), "Takeoff complete. Flying to start of Row 1.");
            state_ = MissionState::FLYING_TO_START;
        }
    }

    void handle_fly_to_start()
    {
        // Navigate to start of current row
        if (current_row_ >= static_cast<int>(row_waypoints_.size())) {
            RCLCPP_INFO(get_logger(), "All rows completed! Returning home.");
            state_ = MissionState::RETURNING_HOME;
            return;
        }

        auto& start_point = row_waypoints_[current_row_][0];
        double dx = start_point[0] - current_x_;
        double dy = start_point[1] - current_y_;
        double distance = std::hypot(dx, dy);

        if (distance < 0.5) {
            RCLCPP_INFO(get_logger(), "Reached start of Row %d. Beginning scan.", current_row_ + 1);
            state_ = MissionState::SCANNING_ROW;
            current_waypoint_ = 1;  // Move to end of row
        } else {
            // Simple proportional control to reach waypoint
            auto cmd = geometry_msgs::msg::Twist();
            cmd.linear.x = std::clamp(dx * 0.5, -0.5, 0.5);
            cmd.linear.y = std::clamp(dy * 0.5, -0.5, 0.5);
            cmd.linear.z = std::clamp((target_altitude_ - current_z_) * 1.0, -0.3, 0.3);
            cmd_vel_pub_->publish(cmd);
        }
    }

    void handle_scanning_row()
    {
        // Navigate along the row to the end waypoint
        auto& end_point = row_waypoints_[current_row_][1];
        double dx = end_point[0] - current_x_;
        double dy = end_point[1] - current_y_;
        double distance = std::hypot(dx, dy);

        if (distance < 0.5) {
            RCLCPP_INFO(get_logger(), "Completed Row %d. Trees detected: %d",
                       current_row_ + 1, tree_detection_count_);
            current_row_++;
            state_ = MissionState::MOVING_TO_NEXT_ROW;
        } else {
            // Slow, steady movement for accurate scanning
            auto cmd = geometry_msgs::msg::Twist();
            cmd.linear.x = std::clamp(dx * 0.3, -0.3, 0.3);  // Slower for scanning
            cmd.linear.y = std::clamp(dy * 0.3, -0.3, 0.3);
            cmd.linear.z = std::clamp((target_altitude_ - current_z_) * 1.0, -0.3, 0.3);
            cmd_vel_pub_->publish(cmd);
        }
    }

    void handle_moving_to_next_row()
    {
        if (current_row_ >= static_cast<int>(row_waypoints_.size())) {
            RCLCPP_INFO(get_logger(), "All rows scanned! Returning to home base.");
            state_ = MissionState::RETURNING_HOME;
        } else {
            RCLCPP_INFO(get_logger(), "Moving to Row %d", current_row_ + 1);
            state_ = MissionState::FLYING_TO_START;
        }
    }

    void handle_returning_home()
    {
        double dx = home_x_ - current_x_;
        double dy = home_y_ - current_y_;
        double distance = std::hypot(dx, dy);

        if (distance < 0.5) {
            RCLCPP_INFO(get_logger(), "Reached home base. Landing.");
            state_ = MissionState::LANDING;
        } else {
            auto cmd = geometry_msgs::msg::Twist();
            cmd.linear.x = std::clamp(dx * 0.5, -0.5, 0.5);
            cmd.linear.y = std::clamp(dy * 0.5, -0.5, 0.5);
            cmd.linear.z = std::clamp((target_altitude_ - current_z_) * 1.0, -0.3, 0.3);
            cmd_vel_pub_->publish(cmd);
        }
    }

    void handle_landing()
    {
        if (current_z_ > 0.15) {
            auto cmd = geometry_msgs::msg::Twist();
            cmd.linear.z = -0.2;  // Gentle descent
            cmd_vel_pub_->publish(cmd);
        } else {
            stop_drone();
            RCLCPP_INFO(get_logger(), "Mission completed successfully! Total trees detected: %d",
                       tree_detection_count_);
            state_ = MissionState::COMPLETED;
        }
    }

    void publish_status()
    {
        auto msg = std_msgs::msg::String();

        switch (state_) {
            case MissionState::IDLE:
                msg.data = "Status: IDLE - Ready to start";
                break;
            case MissionState::TAKING_OFF:
                msg.data = "Status: TAKING OFF";
                break;
            case MissionState::FLYING_TO_START:
                msg.data = "Status: FLYING TO ROW " + std::to_string(current_row_ + 1);
                break;
            case MissionState::SCANNING_ROW:
                msg.data = "Status: SCANNING ROW " + std::to_string(current_row_ + 1) +
                          " (Trees: " + std::to_string(tree_detection_count_) + ")";
                break;
            case MissionState::MOVING_TO_NEXT_ROW:
                msg.data = "Status: MOVING TO NEXT ROW";
                break;
            case MissionState::RETURNING_HOME:
                msg.data = "Status: RETURNING HOME";
                break;
            case MissionState::LANDING:
                msg.data = "Status: LANDING";
                break;
            case MissionState::COMPLETED:
                msg.data = "Status: MISSION COMPLETE (Trees: " +
                          std::to_string(tree_detection_count_) + ")";
                break;
            case MissionState::EMERGENCY_STOP:
                msg.data = "Status: EMERGENCY STOP";
                break;
        }

        status_pub_->publish(msg);
    }

    std::string state_to_string(MissionState state)
    {
        switch (state) {
            case MissionState::IDLE: return "IDLE";
            case MissionState::TAKING_OFF: return "TAKING_OFF";
            case MissionState::FLYING_TO_START: return "FLYING_TO_START";
            case MissionState::SCANNING_ROW: return "SCANNING_ROW";
            case MissionState::MOVING_TO_NEXT_ROW: return "MOVING_TO_NEXT_ROW";
            case MissionState::RETURNING_HOME: return "RETURNING_HOME";
            case MissionState::LANDING: return "LANDING";
            case MissionState::COMPLETED: return "COMPLETED";
            case MissionState::EMERGENCY_STOP: return "EMERGENCY_STOP";
            default: return "UNKNOWN";
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ForestMissionOrchestrator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
