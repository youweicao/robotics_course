#include<iostream>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class Turtlebot : public rclcpp::Node 
{
private:
    /* data */
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_; 
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;
    void run();

public:
    Turtlebot(const std::string& node_name);
    ~Turtlebot();
};

Turtlebot::Turtlebot(const std::string& node_name) : Node(node_name), counter_(0)
{
    RCLCPP_INFO(this->get_logger(), "Press ctrl+c to terminate");

    // publisher for cmd_vel topic 

    vel_pub_= this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Turtlebot::run, this));
}

Turtlebot::~Turtlebot()
{
}

void Turtlebot::run()
{
    geometry_msgs::msg::Twist vel;


    if (counter_>0 && counter_<20)
    {
        vel.linear.x = 0.5;
        vel.linear.y = 0.0;
    } 
    else if (counter_>=20 && counter_<100)
    {
        vel.linear.x = 0.5;
        vel.linear.y = 0.0;
    }
    else if (counter_>=100 && counter_<180)
    {
        vel.linear.x = 0.0;
        vel.linear.y = 0.5;
    }
    else if (counter_>=180 && counter_<260)
    {
        vel.linear.x = -0.5;
        vel.linear.y = 0.0;
    }
    else if (counter_>=260 && counter_<340)
    {
        vel.linear.x = 0.0;
        vel.linear.y = -0.5;
    }
    else
    {
        vel.linear.x = 0.0;
        vel.linear.y = 0.0;
    }

    vel_pub_->publish(vel);
    counter_++;

}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Turtlebot>("turtlebot_move");
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}