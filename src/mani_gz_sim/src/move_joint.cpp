#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace std::chrono_literals;

class MoveJoint : public rclcpp::Node
{
public:
    MoveJoint()
    : Node("move_joint"), command_sent_(false)
    {
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
                "/joint_trajectory_controller/joint_trajectory", 10);

        timer_ = this->create_wall_timer(500ms,
            std::bind(&MoveJoint::send_command, this));

        RCLCPP_INFO(
            this->get_logger(),
            "Waiting for the joint trajectory controller...");
    }

private:
    void send_command()
    {
        // Wait until the controller subscribes to the topic.
        if (publisher_->get_subscription_count() == 0) {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Waiting for the controller subscription...");
            return;
        }

        if (command_sent_) {
            return;
        }

        trajectory_msgs::msg::JointTrajectory trajectory;

        // Joint positions must follow the same order as joint_names.
        trajectory.joint_names = {"joint_1", "joint_2"};

        trajectory_msgs::msg::JointTrajectoryPoint point;

        // Target positions in radians:
        // joint_1 = 90 degrees; joint_2 = 0 degrees.
        point.positions = {1.5708, 0.0};

        // Request the target position in 3 seconds.
        point.time_from_start.sec = 3;
        point.time_from_start.nanosec = 0;

        trajectory.points.push_back(point);

        publisher_->publish(trajectory);

        RCLCPP_INFO(
            this->get_logger(),
            "Published trajectory: joint_1 = 90 degrees, joint_2 = 0 degrees.");

        command_sent_ = true;
        timer_->cancel();
    }

    rclcpp::Publisher<
        trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;

    rclcpp::TimerBase::SharedPtr timer_;

    bool command_sent_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MoveJoint>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}