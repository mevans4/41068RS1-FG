#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class DroneController : public rclcpp::Node
{
public:
    DroneController() : Node("drone_controller")
    {
        vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&DroneController::run, this));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry", 1000, std::bind(&DroneController::odoCallback,this,_1));
        goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&DroneController::goalPoseCallback, this, std::placeholders::_1));
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/path", 10,
            std::bind(&DroneController::pathCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Drone controller node started.");
        RCLCPP_INFO(this->get_logger(), "Subscribed to /path for dynamic waypoints.");
    }

private:
    bool first_run_ = true;

    void odoCallback(const nav_msgs::msg::Odometry& msg)
    { 
        pose_ = msg;
    }

    nav_msgs::msg::Odometry getOdometry(void)
    {
        return pose_;
    }

    void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        latest_goal_pose_ = *msg;
        RCLCPP_INFO(this->get_logger(), "Received goal pose: x=%.2f, y=%.2f, z=%.2f",
                    msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    }

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (msg->poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty path!");
            return;
        }

        // Clear existing waypoints and populate from path
        planned_waypoints_.clear();
        for (const auto& pose_stamped : msg->poses) {
            Coordinate waypoint;
            waypoint.x = pose_stamped.pose.position.x;
            waypoint.y = pose_stamped.pose.position.y;
            waypoint.z = pose_stamped.pose.position.z;
            waypoint.roll = 0.0;
            waypoint.pitch = 0.0;
            waypoint.yaw = 0.0;
            planned_waypoints_.push_back(waypoint);
        }

        // Reset waypoint index to start following new path
        current_waypoint_index_ = 0;
        has_planned_path_ = true;

        RCLCPP_INFO(this->get_logger(), "Received path with %zu waypoints. Starting navigation.",
                    planned_waypoints_.size());
    }

    void run()
    {
        // Wait for odometry update only on the first run
        if (first_run_) {
            RCLCPP_INFO(this->get_logger(), "Waiting for odometry update...");
            rclcpp::sleep_for(std::chrono::seconds(5));
            first_run_ = false;
        }

        current_pose = getOdometry();

        auto msg = geometry_msgs::msg::Twist();

        // Use planned waypoints if available, otherwise use hardcoded fallback
        std::vector<Coordinate> waypoints;
        if (has_planned_path_ && !planned_waypoints_.empty()) {
            waypoints = planned_waypoints_;
        } else {
            // Fallback to hardcoded waypoints
            Coordinate goal1 = {4, 26, 0.5, 0, 0, 1.57};
            Coordinate goal2 = {8, 26, 0.5, 0, 0, -1.57};
            waypoints = {goal1, goal2};
        }

        // Check if we've reached all waypoints
        if (current_waypoint_index_ >= waypoints.size()) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                               "All waypoints reached. Hovering.");
            msg.linear.x = 0.0;
            msg.linear.y = 0.0;
            msg.linear.z = 0.0;
            sendCmd(msg);
            return;
        }

        Coordinate current_goal = waypoints[current_waypoint_index_];

        double dx = current_goal.x - current_pose.pose.pose.position.x;
        double dy = current_goal.y - current_pose.pose.pose.position.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Waypoint %zu/%zu - Current: (%.2f, %.2f, %.2f), "
                           "Target: (%.2f, %.2f, %.2f), Distance: %.2f",
                           current_waypoint_index_ + 1, waypoints.size(),
                           current_pose.pose.pose.position.x,
                           current_pose.pose.pose.position.y,
                           current_pose.pose.pose.position.z,
                           current_goal.x, current_goal.y, current_goal.z,
                           distance);

        if (distance < 0.3) // Threshold to consider waypoint reached
        {
            RCLCPP_INFO(this->get_logger(), "Waypoint %zu reached! Moving to next.",
                       current_waypoint_index_ + 1);
            current_waypoint_index_++;

            // If we just finished the last waypoint, stop
            if (current_waypoint_index_ >= waypoints.size()) {
                msg.linear.x = 0.0;
                msg.linear.y = 0.0;
                msg.linear.z = 0.0;
            }
        }
        else
        {
            // Simple proportional controller to move towards the waypoint
            msg.linear.x = (dx / distance) * 0.5; // Scale speed
            msg.linear.y = (dy / distance) * 0.5;
            msg.linear.z = 0.0; // Maintain current altitude

            // Yaw control can be added here if needed
            msg.angular.x = 0.0;
            msg.angular.y = 0.0;
            msg.angular.z = 0.0;
        }

        sendCmd(msg);
    }

    // void publish_command()
    // {
    //     auto msg = geometry_msgs::msg::Twist();

    //     // Hover command: Apply slight upward force to maintain altitude
    //     msg.linear.x = 0.3;
    //     msg.linear.y = 0.0;
    //     msg.linear.z = 0.0;  // Adjust this value depending on gravity & tuning

    //     msg.angular.x = 0.0;
    //     msg.angular.y = 0.0;
    //     msg.angular.z = 0.0;

    //     sendCmd(msg);
    // }

    void sendCmd(const geometry_msgs::msg::Twist& cmd)
    {
        vel_publisher_->publish(cmd);
    }

    struct Coordinate
    {
        double x;
        double y;
        double z;
        double roll;
        double pitch;
        double yaw;
    };

    int height_ = 0.5;
    geometry_msgs::msg::PoseStamped latest_goal_pose_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    nav_msgs::msg::Odometry current_pose;
    nav_msgs::msg::Odometry pose_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Path following state
    std::vector<Coordinate> planned_waypoints_;
    size_t current_waypoint_index_ = 0;
    bool has_planned_path_ = false;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneController>());
    rclcpp::shutdown();
    return 0;
}
