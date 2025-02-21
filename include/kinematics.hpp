#pragma once
#include <cmath>
#include <iostream>


namespace NP1_Kin
{
    constexpr float KIN_PI = 3.1415;
    constexpr float R = .009;  // Replace with your R value
    constexpr float STALL_TORQUE = .0036;
    constexpr float MAX_TORQUE = .04;
    constexpr float jacobian_ang[6] = {-0.33333f, -0.33333f, 0.33333f, -0.5f, 0.5f, 0.0f}; // 2x3

    // constexpr float jacobian_tor[6] = {0};
    constexpr float A_dagger[6] = {-37.037037, -55.555555, -37.037037, 55.555555, 37.037037, 0.};

    // helper: torque to tendon force
    float* f_tendon(float tor0, float tor1);
    // helper: force offset
    void f_offset(float* tendon);

    // joint toruqe to motor torque
    float* torque_j2m(float torque1, float torque2);

    // motor angle to joint angle
    float* angle_m2j(float ang1, float ang2, float ang3);

    // radian to degree
    float RadToDeg(float ang);

    // degree to radian
    float DegToRad(float ang);

    // revolutions to degrees
    float RevToDeg(float encoder);
}
