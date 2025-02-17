#pragma once

#include <Arduino.h>
#include "odrive_can.hpp"

class PVPID {
public:
    // Constructor with default gains
    PVPID(float pos_kp = 1.0f, float pos_ki = 0.0f, float pos_kd = 0.0f,
          float vel_kp = 0.1f, float vel_ki = 0.0f, float vel_kd = 0.0f);

    // Set target position and velocity
    void setTargets(float position, float velocity);

    // Update control loop and get torque command
    float update(float current_position, float current_velocity);

    // Reset internal state (integrators, etc.)
    void reset();

    // Gain setters
    void setPositionGains(float kp, float ki, float kd);
    void setVelocityGains(float kp, float ki, float kd);

    // Limit setters
    void setTorqueLimits(float min_torque, float max_torque);
    void setIntegratorLimits(float pos_limit, float vel_limit);

private:
    // PID gains
    float pos_kp_, pos_ki_, pos_kd_;  // Position loop gains
    float vel_kp_, vel_ki_, vel_kd_;  // Velocity loop gains

    // Target values
    float target_position_;
    float target_velocity_;

    // Previous errors for derivative calculation
    float prev_pos_error_;
    float prev_vel_error_;

    // Integral terms
    float pos_integral_;
    float vel_integral_;

    // Previous time for dt calculation
    unsigned long prev_time_;

    // Limits
    float min_torque_;
    float max_torque_;
    float pos_integral_limit_;
    float vel_integral_limit_;

    // Helper function to limit a value between min and max
    float limit(float value, float min_val, float max_val);
};