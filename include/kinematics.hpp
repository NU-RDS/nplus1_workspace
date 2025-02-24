#pragma once
#include <cmath>
#include <vector>
#include <iostream>
#include <algorithm>


namespace NP1_Kin
{
    // Mechanical design constants
    constexpr float KIN_PI = 3.1415;
    constexpr float R_motor = 0.009;
    constexpr float R_joint = 0.00858;
    constexpr float STALL_TORQUE = 0.0;
    constexpr float MAX_TORQUE = .006;
    constexpr float GEAR_RATIO = 22.6;

    // Kinematics matrices
    const std::vector<std::vector<float>> transmission_mat = {
        {-0.369075369075369, 0.369075369075369, 0.369075369075369},
        {0.0f, 0.553613053613054, -0.553613053613054}
    };
    const std::vector<std::vector<float>> RAT = {
        {-0.903157894736842, 0.},
        {0.903157894736842, 0.903157894736842},
        {0.903157894736842, -0.903157894736842}
    };
    
    const std::vector<std::vector<float>> A_pinv = {
        {-38.85003885, 0.},
        {38.85003885, 58.2750582751},
        {38.85003885, -58.2750582751}
    };


    // Torque calculation functiona
    std::vector<float> calcTendonForce(float proximal_torque, float distal_torque);
    std::vector<float> tensionToMotorTorque(const std::vector<float>& tendon_tensions);
    std::vector<float> offsetTensions(const std::vector<float>& tendon_tensions);
    std::vector<float> scaleTorque(const std::vector<float>& motor_torques);

    // Angle conversion functions
    std::vector<float> motorAngleToJoint(float motor_angle_0, float motor_angle_1, float motor_angle_2);
    std::vector<float> jointAngleToMotor(float proximal_angle, float distal_angle);

    // radian to degree
    float RadToDeg(float ang);

    // degree to radian
    float DegToRad(float ang);

    // revolutions to degrees
    float RevToDeg(float encoder);

    // motor angle to shaft angle
    float toShaft(float ang);
}