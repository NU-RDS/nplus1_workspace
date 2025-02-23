#include <cmath>
#include <vector>
#include "kinematics.hpp"

using namespace std;

namespace NP1_Kin
{

    std::vector<float> offsets(3);


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
            if (i > 0)
            {
                temp = 0. - tendon[i];
            }
            else
            {
                temp = (0. + tendon[i])/2;
            }
            if (temp > alpha)
            {
                alpha = temp;
            }
        }
        // after we calculated alpha
        for (int i = 0; i < 3; i++)
        {
            if (i > 0)
            {
                tendon[i] = tendon[i] + alpha;
            }
            else
            {
                tendon[i] = tendon[i] - 2 * alpha;
            }
        }

    }


    // joint toruqe to motor torque
    float* torque_j2m(float* force_tendon)
    {
        static float motor_torques[3] = {0.f};  // Keep only this declaration
    
        // // check if need offset
        bool positive_force = true;
        for (int i = 0; i < 3; i++)
        {
            if (i ==0) {
                positive_force = positive_force && ((force_tendon[i] * -1.) > 0.);
            }
            else {
                positive_force = positive_force && (force_tendon[i] > 0.);
            }
        }
    
        // // any tenson force not positive
        if (!positive_force)
        {
            f_offset(force_tendon);
        }

        for (int i = 0; i < 3; i++) {
            motor_torques[0] = toShaft(force_tendon[i] * R_motor);
        }

        // float max_val = *max_element(motor_torques, motor_torques+3);
    
        // // now all force on tendon should be positive
        // // calculate for motor torque 

        // float scale =  0.;
        // if (abs(max_val) > MAX_TORQUE) {
        //     scale = max_val / MAX_TORQUE;
        // }

        // if (scale != 0.) {
        //     for (int i = 0; i < 3; i++) {
        //         motor_torques[0] = motor_torques[0]*scale;
        //     }
        // }
        

        for (int i = 0; i < 3; i++)
        {
           if (abs(motor_torques[i]) < STALL_TORQUE)
           {
                if (i == 0) {
                    motor_torques[i] = -STALL_TORQUE;
                }
                else {
                    motor_torques[i] = STALL_TORQUE;
                }
           }
           if (abs(motor_torques[i]) > MAX_TORQUE)
           {
                if (i == 0) {
                    motor_torques[i] = -MAX_TORQUE;
                }
                else {
                    motor_torques[i] = MAX_TORQUE;
                }
           }
        }
    
        return motor_torques;
    }

    // motor angle to joint angle
    std::vector<float> angle_m2j(float ang0, float ang1, float ang2)
    {
        std::vector<float> angle_joint;

        angle_joint.push_back(jacobian_ang[0] * ang0 + jacobian_ang[1] * ang1 + jacobian_ang[2] * ang2); // pip
        angle_joint.push_back(jacobian_ang[3] * ang0 + jacobian_ang[4] * ang1 + jacobian_ang[5] * ang2); // dip

        return angle_joint;
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