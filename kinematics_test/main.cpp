#include "kinematics.hpp"
#include <iostream>

using namespace std;
using namespace NP1_Kin;

int main()
{
    // init joint torque
    float torque0 = 0.1f; 
    float torque1 = 0.1f; 
    cout << "joint torque 0: " << torque0 << endl;
    cout << "joint torque 1: " << torque1 << endl;

    // init motor torque
    static float motor_torques[3] = {0};  

    // Get tendon forces
    float* tendon = f_tendon(torque0, torque1);
    
    cout << "--------------------------------------" << endl;
    cout << "Tendon forces:" << endl;
    cout << "Tendon 0: " << tendon[0] << endl;
    cout << "Tendon 1: " << tendon[1] << endl;
    cout << "Tendon 2: " << tendon[2] << endl;
    
    bool positive_force = true;
    for (int i = 0; i < 3; i++)
    {
        positive_force = positive_force && (tendon[i] > 0.);
    }

    // any tension force not positive
    if (!positive_force)
    {
        // Pass individual array elements instead of the pointer
        f_offset(tendon);
        cout << "--------------------------------------" << endl;
        cout << "Offset forces:" << endl;
        cout << "Tendon 0: " << tendon[0] << endl;
        cout << "Tendon 1: " << tendon[1] << endl;
        cout << "Tendon 2: " << tendon[2] << endl;
    }
    else
    {
        cout << "no force offset" << endl;
    }

    // motor torque
    cout << "--------------------------------------" << endl;
    for (int i = 0; i < 3; i++)
    {
        float motor_torque = tendon[i] * R;
        if (motor_torque < STALL_TORQUE)
        {
            motor_torque = STALL_TORQUE;
        }
        if (motor_torque > MAX_TORQUE)
        {
            motor_torque = MAX_TORQUE;
        }
        motor_torques[i] = motor_torque;  // Store directly in the static array
        cout << "Motor " << i << " torque: " << motor_torque << endl;
        
    }

    cout << "--------------------------------------" << endl;
    float* m_torque = torque_j2m (torque0, torque1);
    cout << "With torque_j2m: " << endl;
    cout << "Motor 0: " << m_torque[0] << endl;
    cout << "Motor 1: " << m_torque[1] << endl;
    cout << "Motor 2: " << m_torque[2] << endl;

    // motor to joint angle
    float* joint_ang = angle_m2j(1., 2., 3.);
    cout << "--------------------------------------" << endl;
    cout << "Motor to joint angle: " << endl;
    cout << "Joint 0: " << joint_ang[0] << endl;
    cout << "Joint 1: " << joint_ang[1] << endl;

    return 0;
}