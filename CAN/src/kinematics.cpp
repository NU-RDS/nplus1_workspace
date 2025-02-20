#pragma once
#include <cmath>
#include "kinematics.hpp"

namespace NP1_Kin
{

    // helper: torque to tendon force
    float* f_tendon(float tor0, float tor1)
    {
        static float tendon[3];
        tendon[0] = A_dagger[0] * tor0 + A_dagger[1] * tor1; // tendon one motor 0
        tendon[1] = A_dagger[2] * tor0 + A_dagger[3] * tor1; // tendon one motor 1
        tendon[2] = A_dagger[4] * tor0 + A_dagger[5] * tor1; // tendon one motor 2

        return tendon;
    }

    // helper: force offset
    void f_offset(float* tendon)
    {
        float alpha = 0; // records the maximum difference of the negative term(s)
        float temp = 0;
        for (int i = 0; i < 3; i++)
        {
            if (i < 2)
            {
                temp = 0. - tendon[i];
            }
            else
            {
                temp = (0. - tendon[i])/2;
            }
            if (temp > alpha)
            {
                alpha = temp;
            }
        }

        // after we calculated alpha
        for (int i = 0; i < 3; i++)
        {
            if (i < 2)
            {
                tendon[i] = tendon[i] + alpha;
            }
            else
            {
                tendon[i] = tendon[i] + 2 * alpha;
            }
        }

    }


    // joint toruqe to motor torque
    float* torque_j2m(float torque0, float torque1)
    {
        static float motor_torques[3] = {0};
        float* force_tendon = f_tendon(torque0, torque1);

        // check if need offset
        bool positive_force = true;
        for (int i = 0; i < 3; i++)
        {
            positive_force = positive_force && (force_tendon[i] > 0.);
        }

        // any tenson force not positive
        if (!positive_force)
        {
            f_offset(force_tendon);
        }

        // now all force on tendon should be positive
        // calculate for motor torque 
        float* motor_torques[3];
        for (int i = 0; i < 3; i++)
        {
           float motor_torque = force_tendon[i] * R;
           if (motor_torque < STALL_TORQUE)
           {
            motor_torque = STALL_TORQUE;
           }
           if (motor_torque > MAX_TORQUE)
           {
            motor_torque = MAX_TORQUE;
           }
           motor_torques[i] = motor_torque;
        }

        return motor_torques;
    }

    // motor angle to joint angle
    float* angle_m2j(float ang0, float ang1, float ang2)
    {
        static float angle_joint[2];

        angle_joint[0] = jacobian_ang[0] * ang0 + jacobian_ang[1] * ang1 + jacobian_ang[2] * ang2; // dip
        angle_joint[1] = jacobian_ang[3] * ang0 + jacobian_ang[4] * ang1 + jacobian_ang[5] * ang2; // pip

        return angle_joint;
    }

    // radian to degree
    float radToDeg(float ang)
    {
        return ang / PI * 180.;
    }

    // degree to radian
    float DegToRad(float ang)
    {
        return ang * PI / 180.;
    }
}


