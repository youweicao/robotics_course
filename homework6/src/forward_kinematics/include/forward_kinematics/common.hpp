#ifndef FORWARD_KINEMATICS__COMMON_HPP_
#define FORWARD_KINEMATICS__COMMON_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/header.hpp"
#include <array>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
std::array<double, 3> forward_kinematics(const std::array<double, 3>& joint_angles);
#endif  // FORWARD_KINEMATICS__COMMON_HPP_
