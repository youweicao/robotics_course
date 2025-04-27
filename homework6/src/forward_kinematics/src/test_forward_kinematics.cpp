#include "forward_kinematics/common.hpp"

class Manipulator : public rclcpp::Node
{
public:
    Manipulator() : Node("manipulator")
    {
        RCLCPP_INFO(this->get_logger(), "Press Ctrl + C to terminate");

        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/rx150/joint_states", 10);

        // 初始化 JointState 消息
        joint_msg_.header = std_msgs::msg::Header();
        joint_msg_.name = {"waist", "shoulder", "elbow", "wrist_angle",
                           "wrist_rotate", "gripper", "left_finger", "right_finger"};
        joint_msg_.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.026, -0.026};

        // 测试前向运动学
        test_case_ = {M_PI/6, -M_PI/3, -M_PI/6};
        auto position = forward_kinematics(test_case_);

        RCLCPP_INFO(this->get_logger(), "Joint Angle Test Case: [%.3f, %.3f, %.3f]", 
                    test_case_[0], test_case_[1], test_case_[2]);
        RCLCPP_INFO(this->get_logger(), "End Position: [%.3f, %.3f, %.3f]", 
                    position[0], position[1], position[2]);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&Manipulator::publish_joint_state, this));
    }

private:
    void publish_joint_state()
    {
        joint_msg_.header.stamp = this->now();

        // 更新前3个关节角度
        for (size_t i = 0; i < 3; ++i) {
            joint_msg_.position[i] = test_case_[i];
        }

        joint_pub_->publish(joint_msg_);
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::JointState joint_msg_;
    std::array<double, 3> test_case_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Manipulator>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
