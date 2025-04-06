#include <chrono>
#include <memory>
#include <vector>
#include <fstream>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

class Controller {
    public:
        Controller(double P = 0.0, double D = 0.0, double set_point = 0.0) : Kp_(P), Kd_(D), set_point_(set_point), previous_error_(0.0) {}
    
        double update(double current_value) {
            double error = set_point_ - current_value;
            double derivative = error - previous_error_;
            previous_error_ = error;
    
            double P_term = Kp_ * error;
            double D_term = Kd_ * derivative;
    
            return P_term + D_term;
        }
    
        void setPoint(double set_point) {
            set_point_ = set_point;
            previous_error_ = 0.0;
        }
    
        void setPD(double P, double D) {
            Kp_ = P;
            Kd_ = D;
        }
    
    private:
        double Kp_;
        double Kd_;
        double set_point_;
        double previous_error_;
};
    

class Turtlebot : public rclcpp::Node {
public:
    Turtlebot() : Node("turtlebot_move"), reset_count_(10), logging_counter_(0), pid_theta_(1.0, 0.2, 0.0)  // PD control
    {
        RCLCPP_INFO(this->get_logger(), "Press Ctrl + C to terminate");

        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        reset_pub_ = this->create_publisher<std_msgs::msg::Empty>("reset_odometry", 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&Turtlebot::odom_callback, this, _1));

        timer_ = this->create_wall_timer(100ms, std::bind(&Turtlebot::publish_reset, this));
    }

    ~Turtlebot() {
        // std::ofstream file("trajectory.csv");
        // for (const auto& point : trajectory_) {
        //     file << point[0] << "," << point[1] << "\n";
        // }
        // file.close();
        // RCLCPP_INFO(this->get_logger(), "Trajectory saved.");
    }
    void run() {
        move_to_point(0,0);

        move_to_point(1,0);
        move_to_point(5,0);
        move_to_point(5,4);
        move_to_point(1,4);
        move_to_point(1,0);
    }

private:
    void publish_reset() {
        if (reset_count_ > 0) {
            std_msgs::msg::Empty msg;
            reset_pub_->publish(msg);
            reset_count_--;
        } else {
            timer_->cancel();
            RCLCPP_INFO(this->get_logger(), "Odometry reset.");
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );

        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        pose_.x = msg->pose.pose.position.x;
        pose_.y = msg->pose.pose.position.y;
        pose_.theta = yaw;

        logging_counter_++;
        if (logging_counter_ == 100) {
            logging_counter_ = 0;
            trajectory_.push_back({pose_.x, pose_.y});
            RCLCPP_INFO(this->get_logger(), "odom: x=%.2f; y=%.2f; theta=%.2f", pose_.x, pose_.y, yaw);
        }
    }

    double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    void turn_to_angle(double target_angle, double tolerance = 0.01) {
        pid_theta_.setPoint(target_angle);

        double error = normalize_angle(target_angle - pose_.theta);
        while (fabs(error) > tolerance) {
            double control_output = pid_theta_.update(pose_.theta);
            vel_.linear.x = 0.0;
            vel_.angular.z = control_output;
    
            vel_pub_->publish(vel_);
            rclcpp::sleep_for(100ms);
    
            error = target_angle - pose_.theta;
        }
        stop();
    }

    void drive_straight(double target_distance, double tolerance = 0.01) {
        double initial_x = pose_.x;
        double initial_y = pose_.y;
        double distance_travelled = 0.0;

        pid_theta_.setPoint(target_distance);

        while ((target_distance - distance_travelled) > tolerance) {
            distance_travelled = sqrt(pow(pose_.x - initial_x, 2) + pow(pose_.y - initial_y, 2));
            double control_output = pid_theta_.update(distance_travelled);
            vel_.linear.x = control_output;
            vel_.angular.z = 0.0;

            vel_pub_->publish(vel_);
            rclcpp::sleep_for(100ms);
        }
        stop(); 
    }

    void move_to_point(double x, double y, double tolerance = 0.1) {
        double target_angle = atan2(y - pose_.y, x - pose_.x); 
        turn_to_angle(target_angle);

        double target_distance = sqrt(pow(x - pose_.x, 2) + pow(y - pose_.y, 2));
        drive_straight(target_distance, tolerance);
    }

    void stop() {
        vel_.linear.x = 0.0;
        vel_.angular.z = 0.0;
        vel_pub_->publish(vel_); 
        RCLCPP_INFO(this->get_logger(), "Turtlebot stopped.");
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr reset_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::Pose2D pose_;
    std::vector<std::vector<double>> trajectory_;

    int reset_count_;
    int logging_counter_;

    geometry_msgs::msg::Twist vel_; 
    Controller pid_theta_; 
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Turtlebot>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor](){ executor.spin();});

    rclcpp::sleep_for(1000ms);
    node->run();

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}
