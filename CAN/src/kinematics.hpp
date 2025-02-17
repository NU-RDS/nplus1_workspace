#pragma once
#include <cmath>

namespace NP1_Kin
{
    constexpr double PI = 3.1415;
    constexpr double R = 9.0;  // Replace with your R value
    
    constexpr float jacobian[6] = {-0.333f, -0.333f, 0.333f, -0.5f, 0.5f, 0.0f};

    // joint toruqe to motor torque
    float* torque_j2m(float torque1, float torque2);

    // motor angle to joint angle
    float* angle_m2j(float ang1, float ang2, float ang3);

    // radian to degree
    float radToDeg(float ang);

    // degree to radian
    float DegToRad(float ang);
}


