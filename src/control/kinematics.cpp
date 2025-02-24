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
            motor_torques[i] = toShaft(tendon_tensions[i] * R_motor);
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
            if (temp > alpha)
            {
                alpha = temp;
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
        // Initialize scaled_torques
        std::vector<float> scaled_torques(3);

        // Find the maximum absolute value among the torque values
        float maxValue = 0.0;
        for (size_t i = 0; i < motor_torques.size(); i++) {
            maxValue = std::max(maxValue, motor_torques[i]);
        }
        
        // Check if scaling is needed
        if (maxValue > MAX_TORQUE) {
            // Calculate the scaling factor
            float scaleFactor = MAX_TORQUE / maxValue;
            
            // Scale all torques proportionally
            for (size_t i = 0; i < motor_torques.size(); i++) {
                scaled_torques[i] = motor_torques[i]*scaleFactor;
            }
        }
        
        return scaled_torques;
    }

    // motor angle to joint angle
    std::vector<float> motorAngleToJoint(float motor_angle_0, float motor_angle_1, float motor_angle_2)
    {
        std::vector<float> joint_angles(2);
        
        for (size_t i = 0; i < jacobian_ang.size(); i++){
            joint_angles[i] = jacobian_ang[i][0] * motor_angle_0 + jacobian_ang[i][1] * motor_angle_1 + jacobian_ang[i][2] * motor_angle_2;
        }

        return joint_angles;
    }

    std::vector<float> jointAngleToMotor(float proximal_angle, float distal_angle)
    {
        std::vector<float> motor_angles(3);

        for (size_t i = 0; i < jacobian_ang[0].size(); i++) {
            motor_angles[i] = jacobian_ang[0][i] * proximal_angle + jacobian_ang[1][i] * distal_angle;
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