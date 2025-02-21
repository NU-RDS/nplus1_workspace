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
JointPIDController::JointPIDController(float p_gain, float i_gain, float d_gain, 
                                        float max_t, float min_t)
        : kp(p_gain), ki(i_gain), kd(d_gain),
        integral_error(0.0), prev_error(0.0),
        max_torque(max_t), min_torque(min_t) {
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
    return std::max(min_torque, std::min(max_torque, output));
}

// Constructor for FingerController
FingerController::FingerController(
    float prox_p, float prox_i, float prox_d,
    float dist_p, float dist_i, float dist_d,
    float max_torque, float min_torque)
    : proximal_joint(prox_p, prox_i, prox_d, max_torque, min_torque),
      distal_joint(dist_p, dist_i, dist_d, max_torque, min_torque) {}

// Compute control torques
std::vector<float> FingerController::computeTorques(
    const std::vector<float>& target_angles,
    const std::vector<float>& current_angles,
    const std::vector<float>& feedforward) {
    
    if (target_angles.size() != 2 || current_angles.size() != 2) {
        std::cout << "Both target and current angles must have size 2";
    }
    
    std::vector<float> torques(2);
    torques[0] = proximal_joint.computeTorque(
        target_angles[0], current_angles[0], 
        feedforward.size() > 0 ? feedforward[0] : 0.0);
    
    torques[1] = distal_joint.computeTorque(
        target_angles[1], current_angles[1],
        feedforward.size() > 1 ? feedforward[1] : 0.0);
    
    return torques;
}

// Reset both joint controllers
void FingerController::reset() {
    proximal_joint.reset();
    distal_joint.reset();
}

// Example usage
int main() {
    // Initialize controller with PID gains
    FingerController finger(
        1.0, 0.1, 0.05,  // Proximal joint PID gains
        0.8, 0.08, 0.04, // Distal joint PID gains
        2.0,  // Max torque
        -2.0  // Min torque
    );
    
    // Target angles (in radians)
    std::vector<float> targets = {M_PI/4, M_PI/3};
    
    // Current angles (in radians)
    std::vector<float> current = {0.0, 0.0};
    
    // Optional feedforward terms
    std::vector<float> feedforward = {0.1, 0.1};
    
    // Compute control torques
    std::vector<float> torques = finger.computeTorques(targets, current, feedforward);
    
    return 0;
}