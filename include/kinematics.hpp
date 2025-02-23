#pragma once
#include <cmath>
#include <vector>
#include <iostream>
#include <algorithm>


namespace NP1_Kin
{
    constexpr float KIN_PI = 3.1415;
    constexpr float R_motor = .009;  // Replace with your R value
    constexpr float R_joint = 0.00858;
    constexpr float STALL_TORQUE = 0.0;
    constexpr float MAX_TORQUE = .03;
    constexpr float jacobian_ang[6] = {-0.33333f, 0.33333f, 0.33333f, 0.0f, 0.5f, -0.5f}; // 2x3
    constexpr float GEAR_RATIO = 22.6;
    // constexpr float jacobian_tor[6] = {0};
    constexpr float A_dagger[6] = {-38.85003885, 0., 38.85003885, 58.2750582751, 38.85003885, -58.2750582751}; // changed to reflect motor numbers not joint numbers

    // helper: torque to tendon force
    float* f_tendon(float tor0, float tor1);
    // helper: force offset
    void f_offset(float* tendon);

    // joint toruqe to motor torque
    float* torque_j2m(float* force_tendon);

    // motor angle to joint angle
    std::vector<float> angle_m2j(float ang1, float ang2, float ang3);

    // radian to degree
    float RadToDeg(float ang);

    // degree to radian
    float DegToRad(float ang);

    // revolutions to degrees
    float RevToDeg(float encoder);

    // motor angle to shaft angle
    float toShaft(float ang);
}