#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
using namespace std::chrono_literals;
 
// Define constants
const std::vector<std::string> rotation_joints = {
    "q1",
    "q2",
    "q3"
};
// const std::vector<double> q_final {-1.0472, 0.0, 1.0472, 0.1, -0.1, 0.1, 0.3426, -0.3426, 0.3426};
// const std::vector<double> q_start {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
// const std::vector<std::string> wheel_joints = {
//     "q10",
//     "q11"
// };
 
class WheelDrivePublisher : public rclcpp::Node
{
public:
    WheelDrivePublisher()
        : Node("wheel_driver")
    {
        // Create the publisher of the desired leg and wheel goal poses
        // leg_pose_publisher_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("/rotation_controller/joint_trajectory", 1);
        wheel_pose_publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>("/wheel_controller/cmd_vel", 10);
 
        // Create a timer to periodically call the timerCallback function
        timer_ = create_wall_timer(0.02s, std::bind(&WheelDrivePublisher::timerCallback, this));
 
        frame_id_ = "base";
 
        // Desired time from the trajectory start to arrive at the trajectory point.
        // Needs to be less than or equal to the timer period above to allow
        // the robotic leg to smoothly transition between points.
        duration_sec_ = 0.015;
        duration_nanosec_ = duration_sec_ * 1e9;  // (seconds * 1e9)
        // q = {0, 0 , 0};
        // // Set the desired goal poses for the robotic leg.
        // // Keep track of the current trajectory we are executing
        index_ = 0;
    }
 
private:
    void timerCallback()
    {
 
        auto msg_wheel = geometry_msgs::msg::TwistStamped();
        msg_wheel.header.frame_id = frame_id_;
        msg_wheel.header.stamp = this->get_clock()->now();
        // msg_wheel.twist.angular.z = -0.5;
        msg_wheel.twist.linear.x = -0.1;
        wheel_pose_publisher_->publish(msg_wheel);
    }
 
    // Publishers for leg and wheel joint trajectories
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr wheel_pose_publisher_;
 
    // Timer for periodic callback
    rclcpp::TimerBase::SharedPtr timer_;
 
    // Frame ID for the joint trajectories
    std::string frame_id_;
 
    // Duration for each trajectory point
    int duration_sec_;
    int duration_nanosec_;
 
    // Desired goal poses for the robotic leg and wheel
    // Index to keep track of the current trajectory point
    size_t index_;
};
 
int main(int argc, char * argv[])
{
    // Initialize the ROS 2 client library
    rclcpp::init(argc, argv);
 
    // Create an instance of the ExampleJointTrajectoryPublisherCpp node
    auto node = std::make_shared<WheelDrivePublisher>();
 
    // Spin the node to execute the callbacks
    rclcpp::spin(node);
 
    // Shutdown the ROS 2 client library
    rclcpp::shutdown();
 
    return 0;
}