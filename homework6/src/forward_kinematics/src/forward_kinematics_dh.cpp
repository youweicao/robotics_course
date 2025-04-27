#include "forward_kinematics/common.hpp"

std::array<double, 3> forward_kinematics(const std::array<double, 3>& joints)
{
    double link1z = 0.065;
    double link2z = 0.039;
    double link3x = 0.050;
    double link3z = 0.150;
    double link4x = 0.150;

    double joint1 = joints[0];
    double joint2 = joints[1];
    double joint3 = joints[2];

    double x = (link3x * cos(joint2) + link4x * cos(joint2 + joint3)) * cos(joint1);
    double y = (link3x * cos(joint2) + link4x * cos(joint2 + joint3)) * sin(joint1);
    double z = link1z + link2z + link3x * sin(joint2) + link4x * sin(joint2 + joint3);

    return {x, y, z};
}
