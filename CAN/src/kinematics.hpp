#pragma once
#include <cmath>

namespace NP1_Kin
{
    constexpr double PI = 3.1415;
    constexpr double R = 9.0;  // Replace with your R value
    constexpr float STALL_TORQUE = .0036;
    constexpr float MAX_TORQUE = .5;
    constexpr float jacobian_ang[6] = {-0.333f, -0.333f, 0.333f, -0.5f, 0.5f, 0.0f}; // 2x3

    constexpr float jacobian_tor[6] = {0};
    constexpr float A_dagger[6] = {-1/3/R, -1/3/R, 1/3/R, -1/2/R, 1/2/R, 0.f}; // 2x3

    // helper: torque to tendon force
    float* f_tendon(float tor0, float tor1);
    // helper: force offset
    void f_offset(float ang1, float ang2, float ang3);

    // joint toruqe to motor torque
    float* torque_j2m(float torque1, float torque2);

    // motor angle to joint angle
    float* angle_m2j(float ang1, float ang2, float ang3);

    // radian to degree
    float radToDeg(float ang);

    // degree to radian
    float DegToRad(float ang);
}


