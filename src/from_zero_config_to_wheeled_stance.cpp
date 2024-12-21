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
const std::vector<std::string> leg_joints = {
    "q1",
    "q2",
    "q3",
    "q4",
    "q5",
    "q6",
    "q7",
    "q8",
    "q9"
};
const std::vector<double> q_final {-1.0472, 0.0, 1.0472, 0.1, -0.1, 0.1, 0.3426, -0.3426, 0.3426};
const std::vector<double> q_start {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
// const std::vector<std::string> wheel_joints = {
//     "q10",
//     "q11"
// };
 
class WheelDrivePublisher : public rclcpp::Node
{
public:
    WheelDrivePublisher()
        : Node("from_zero_config_to_wheeled_stance")
    {
        // Create the publisher of the desired leg and wheel goal poses
        leg_pose_publisher_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("/leg_controller/joint_trajectory", 1);
        // wheel_pose_publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>("/wheel_controller/cmd_vel", 10);
 
        // Create a timer to periodically call the timerCallback function
        timer_ = create_wall_timer(0.02s, std::bind(&WheelDrivePublisher::timerCallback, this));
 
        frame_id_ = "base";
 
        // Desired time from the trajectory start to arrive at the trajectory point.
        // Needs to be less than or equal to the timer period above to allow
        // the robotic leg to smoothly transition between points.
        duration_sec_ = 0.015;
        duration_nanosec_ = duration_sec_ * 1e9;  // (seconds * 1e9)
        q = q_start;
        // Set the desired goal poses for the robotic leg.
        // Keep track of the current trajectory we are executing
        index_ = 0;
    }
 
private:
    void timerCallback()
    {
        static double t = 0;
        t+=0.015;
        if (t <= 1.5) {
        for (long unsigned int i = 0; i < q.size(); i++) {
            q[i] = (q_final[i]-q_start[i])*sin(M_PI*t/3);
        }
        }
        else if ((t > 2) && (t <= 3.5)) {
            for (long unsigned int i = 0; i < q.size(); i++) {
            q[i] = (q_final[i]-q_start[i])*sin(M_PI*(t-1.5)/2);
        }  
        }
        // Create new JointTrajectory messages for leg and wheel
        auto msg_leg = trajectory_msgs::msg::JointTrajectory();
        msg_leg.header.frame_id = frame_id_;
        msg_leg.joint_names = leg_joints;
 
        // auto msg_wheel = geometry_msgs::msg::TwistStamped();
        // msg_leg.header.frame_id = frame_id_;
        // msg_wheel.twist.linear.z = 10;
 
        // Create JointTrajectoryPoints for leg and wheel
        auto point_leg = trajectory_msgs::msg::JointTrajectoryPoint();
        point_leg.positions = q;
        point_leg.time_from_start = rclcpp::Duration(duration_sec_, duration_nanosec_);
        msg_leg.points.push_back(point_leg);
        leg_pose_publisher_->publish(msg_leg);
 
        // auto point_wheel = trajectory_msgs::msg::JointTrajectoryPoint();
        // // q10 -= desired_linear_speed/0.025;
        // // q11 += desired_linear_speed/0.025;
        // point_wheel.positions = {q10, q11};
        // point_wheel.time_from_start = rclcpp::Duration(duration_sec_, duration_nanosec_);
        // wheel_pose_publisher_->publish(msg_wheel);
    }
 
    // Publishers for leg and wheel joint trajectories
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr leg_pose_publisher_;
    // rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr wheel_pose_publisher_;
 
    // Timer for periodic callback
    rclcpp::TimerBase::SharedPtr timer_;
 
    // Frame ID for the joint trajectories
    std::string frame_id_;
 
    // Duration for each trajectory point
    int duration_sec_;
    int duration_nanosec_;
 
    // Desired goal poses for the robotic leg and wheel
    std::vector<std::vector<double>> leg_positions_;
    std::vector<std::vector<double>> wheel_positions_;
    std::vector<double> q;
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