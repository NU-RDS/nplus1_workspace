#pragma once
#include <cmath>

namespace NP1_Kin
{
    constexpr double PI = 3.1415;

    // A: structure Jacobian 
    // [[-R, -R, R],
    // [-R, -R, 0]]
    constexpr size_t ROWS = 2;
    constexpr size_t COLS = 3;
    constexpr double R = 1.0;  // Replace with your R value
    
    constexpr double jacobian[ROWS][COLS] = {
        {-R, -R, R},
        {-R, -R, 0}
    };

    // joint toruqe to motor torque
    float torque_j2m();

    // radian to degree
    double radToDeg();

    // degree to radian
    double DegToRad();
}