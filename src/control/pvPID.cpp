#include <vector>
#include <cmath>
#include <chrono>
#include <iostream>

#include "pvPID.hpp"

// Initialize timing
void JointPIDController::initTiming() {
    last_update = std::chrono::steady_clock::now();
}

// Constructor
JointPIDController::JointPIDController(float p_gain, float i_gain, float d_gain)
        : kp(p_gain), ki(i_gain), kd(d_gain),
        integral_error(0.0), prev_error(0.0) {
    initTiming();
}


// Reset controller state
void JointPIDController::reset() {
    integral_error = 0.0;
    prev_error = 0.0;
    initTiming();
}

// Compute control output
float JointPIDController::computeTorque(float target_angle, float current_angle, float feedforward) {
    // Get time delta
    auto current_time = std::chrono::steady_clock::now();
    float dt = std::chrono::duration<float>(current_time - last_update).count();
    last_update = current_time;

    // Compute error
    float error = target_angle - current_angle;

    if (error < 0.3) {
        error = 0.0;
    }

    // Update integral term with anti-windup
    integral_error += error * dt;

    // Compute derivative term
    float derivative = dt > 0.0 ? (error - prev_error) / dt : 0.0;
    prev_error = error;

    // Compute PID output
    float output = kp * error + ki * integral_error + kd * derivative;

    // Add feedforward term
    output += feedforward;

    return output;
}

// Constructor for FingerController
FingerController::FingerController(
    float prox_p, float prox_i, float prox_d,
    float dist_p, float dist_i, float dist_d)
    : proximal_joint(prox_p, prox_i, prox_d),
      distal_joint(dist_p, dist_i, dist_d) {}

// Compute control torques
std::vector<float> FingerController::computeTorques(
    const std::vector<float>& target_angles,
    const std::vector<float>& current_angles,
    const std::vector<float>& feedforward) {
    
    if (target_angles.size() != 2 || current_angles.size() != 2) {
        // std::cout << "Both target and current angles must have size 2";
    }

    std::vector<float> control_torques = {
        proximal_joint.computeTorque(target_angles[0], current_angles[0]),
        distal_joint.computeTorque(target_angles[1], current_angles[1])
    };
    
    return control_torques;
}

// Reset both joint controllers
void FingerController::reset() {
    proximal_joint.reset();
    distal_joint.reset();
}
