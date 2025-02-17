#pragma once
#include <cmath>
#include "kinematics.hpp"

namespace NP1_Kin
{
    // joint toruqe to motor torque
    float torque_j2m()
    {
        return;
    }

    // radian to degree
    double radToDeg(double ang)
    {
        return ang * PI / 180.;
    }

    // degree to radian
    double DegToRad(double ang)
    {
        return ang / PI * 180.;
    }
}