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

    // Update integral term with anti-windup
    integral_error += error * dt;

    // Compute derivative term
    float derivative = dt > 0.0 ? (error - prev_error) / dt : 0.0;
    prev_error = error;

    // Compute PID output
    float output = kp * error + ki * integral_error + kd * derivative;

    // Add feedforward term
    output += feedforward;

    // Apply torque limits
    return output;
}

// Constructor for FingerController
FingerController::FingerController(
    float prox_p, float prox_i, float prox_d,
    float dist_p, float dist_i, float dist_d)
    : proximal_joint(prox_p, prox_i, prox_d),
      distal_joint(dist_p, dist_i, dist_d) {}

// Compute control torques
float* FingerController::computeTorques(
    const float target_angles[2],
    const float current_angles[2],
    const float feedforward[2]) {

    float torques[2];
    torques[0] = proximal_joint.computeTorque(
        target_angles[0], current_angles[0], feedforward[0]);
    
    torques[1] = distal_joint.computeTorque(
        target_angles[1], current_angles[1], feedforward[1]);

    return torques;
}

// Reset both joint controllers
void FingerController::reset() {
    proximal_joint.reset();
    distal_joint.reset();
}

