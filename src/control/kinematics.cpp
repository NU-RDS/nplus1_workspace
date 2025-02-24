#include <cmath>
#include <vector>

#include "kinematics.hpp"

using namespace std;

namespace NP1_Kin
{
    // helper: torque to tendon force
    std::vector<float> calcTendonForce(float proximal_torque, float distal_torque)
    {
        std::vector<float> tensions(3);

        // Use A_dagger to calculate tension distribution of each tendon on finger joints
        for (size_t i =0; i < A_dagger.size(); i++){
            tensions[i] = A_dagger[i][0] * proximal_torque + A_dagger[i][1] * distal_torque;
        }

        return tensions;
    }


    std::vector<float> tensionToMotorTorque(const std::vector<float>& tendon_tensions)
    {
        std::vector<float> motor_torques(3);

        // Convert tension values to torques in the motor shaft
        for (int i = 0; i < 3; i++) {
            motor_torques[0] = toShaft(tendon_tensions[i] * R_motor);
        }

        return motor_torques;

    }


    std::vector<float> offsetTensions(const std::vector<float>& tendon_tensions) {
        std::vector<float> tension(3);

        float alpha = 0.f;
        float temp = 0.f;

        for (size_t i = 0; i < tendon_tensions.size(); i++) {
            // If Motor 0, half the tension, else scale by 1
            if (i == 0) {
                temp = (tendon_tensions[i]*-1.f)/2.f;
            }
            else {
                temp = tendon_tensions[i]*-1.f;
            }
        }

        for (size_t i = 0; i < tendon_tensions.size(); i++)
        {
            if (i == 0)
            {
                tension[i] = tendon_tensions[i] + 2 * alpha;
            }
            else
            {
                tension[i] = tendon_tensions[i] + alpha;
            }
        }

        return tension;
    }


    std::vector<float> scaleTorque(const std::vector<float>& motor_torques) {
        float min_original = *std::min_element(motor_torques.begin(), motor_torques.end());
        float max_original = *std::max_element(motor_torques.begin(), motor_torques.end());
    
        std::vector<float> scaled_torques;
    
        for (float torque : motor_torques) {
            float scaled = STALL_TORQUE + ((torque - min_original) / (max_original - min_original)) * (MAX_TORQUE - STALL_TORQUE);
            scaled_torques.push_back(scaled);
        }
    
        return scaled_torques;
    }

    std::vector<float> setMotorTorques(const std::vector<float>& motor_torques) {

    }

    // motor angle to joint angle
    std::vector<float> motorAngleToJoint(float motor_angle_0, float motor_angle_1, float motor_angle_2)
    {
        std::vector<float> joint_angles(2);
        
        for (size_t i = 0; i < jacobian_ang[0].size(); i++){
            joint_angles[i] = jacobian_ang[0][i] * motor_angle_0 + jacobian_ang[1][i] * motor_angle_1 + jacobian_ang[0][i] * motor_angle_2;
        }

        return joint_angles;
    }

    std::vector<float> jointAngleToMotor(float proximal_angle, float distal_angle)
    {
        std::vector<float> motor_angles(3);

        for (size_t i = 0; i < jacobian_ang.size(); i++) {
            motor_angles[i] = jacobian_ang[i][0] * proximal_angle + jacobian_ang[i][1] * distal_angle;
        }

        return motor_angles;
    }

    // radian to degree
    float RadToDeg(float ang)
    {
        return ang / KIN_PI * 180.;
    }

    // degree to radian
    float DegToRad(float ang)
    {
        return ang * KIN_PI / 180.;
    }

    // revolutions to degrees
    float RevToDeg(float encoder)
    {
        return encoder * 360.0f;
    }

    // motor angle to shaft angle
    float toShaft(float ang)
    {
        return ang / GEAR_RATIO;
    }
}