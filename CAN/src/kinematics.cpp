#pragma once
#include <cmath>
#include "kinematics.hpp"

namespace NP1_Kin
{
    // joint toruqe to motor torque
    float* torque_j2m(float torque1, float torque2)
    {
        static float torque_motor[3];

        torque_motor[0] = ;
        

        return torque_motor;
    }

    // motor angle to joint angle
    float* angle_m2j(float ang0, float ang1, float ang2)
    {
        static float angle_joint[2];

        angle_joint[0] = jacobian[0] * ang0 + jacobian[1] * ang1 + jacobian[2] * ang2; // dip
        angle_joint[1] = jacobian[3] * ang0 + jacobian[4] * ang1 + jacobian[5] * ang2; // pip

        return angle_joint;
    }

    // radian to degree
    float radToDeg(float ang)
    {
        return ang * PI / 180.;
    }

    // degree to radian
    float DegToRad(float ang)
    {
        return ang / PI * 180.;
    }
}


