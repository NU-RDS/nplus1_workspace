#ifndef FINGER_PID_CONTROLLER_HPP
#define FINGER_PID_CONTROLLER_HPP

#include <vector>
#include <chrono>
#include <stdexcept>
#include <cmath>

#include "kinematics.hpp"

/**
 * @brief PID controller for a single joint
 * 
 * Implements a PID control loop with anti-windup protection,
 * torque limiting, and feedforward support.
 */
class JointPIDController {
private:
    // PID gains
    float kp;  ///< Proportional gain
    float ki;  ///< Integral gain
    float kd;  ///< Derivative gain

    // Error terms
    float integral_error;  ///< Accumulated integral error
    float prev_error;     ///< Previous error for derivative calculation
    
    // Timing
    std::chrono::steady_clock::time_point last_update;
    
    /**
     * @brief Initialize the timing system
     */
    void initTiming();
    
public:
    /**
     * @brief Construct a new Joint PID Controller
     * 
     * @param p_gain Proportional gain
     * @param i_gain Integral gain
     * @param d_gain Derivative gain
     * @param max_t Maximum torque limit
     * @param min_t Minimum torque limit
     */
    JointPIDController(float p_gain, float i_gain, float d_gain);
    
    /**
     * @brief Reset the controller state
     * 
     * Resets integral and previous error terms, and reinitializes timing
     */
    void reset();
    
    /**
     * @brief Compute the control torque
     * 
     * @param target_angle Desired joint angle in radians
     * @param current_angle Current joint angle in radians
     * @param feedforward Optional feedforward term (default: 0.0)
     * @return double Computed torque within specified limits
     */
    float computeTorque(float target_angle, float current_angle, 
                        float feedforward = 0.0);
};

/**
 * @brief Controller for a 2-joint finger
 * 
 * Manages PID controllers for both proximal and distal joints
 * of a robotic finger.
 */
class FingerController {
private:
    JointPIDController proximal_joint;  ///< Controller for proximal joint
    JointPIDController distal_joint;    ///< Controller for distal joint
    
public:
    /**
     * @brief Construct a new Finger Controller
     * 
     * @param prox_p Proximal joint proportional gain
     * @param prox_i Proximal joint integral gain
     * @param prox_d Proximal joint derivative gain
     * @param dist_p Distal joint proportional gain
     * @param dist_i Distal joint integral gain
     * @param dist_d Distal joint derivative gain
     * @param max_torque Maximum torque limit for both joints
     * @param min_torque Minimum torque limit for both joints
     */
    FingerController(
        float prox_p, float prox_i, float prox_d,
        float dist_p, float dist_i, float dist_d);
    
    /**
     * @brief Compute control torques for both joints
     * 
     * @param target_angles Vector of target angles [proximal, distal] in radians
     * @param current_angles Vector of current angles [proximal, distal] in radians
     * @param feedforward Vector of feedforward terms (optional)
     * @return std::vector<double> Computed torques for both joints
     * @throws std::invalid_argument if input vectors are not size 2
     */
    std::vector<float> computeTorques(
        const std::vector<float>& target_angles,
        const std::vector<float>& current_angles,
        const std::vector<float>& feedforward = {0.0, 0.0});
    
    /**
     * @brief Reset both joint controllers
     */
    void reset();
};

#endif // FINGER_PID_CONTROLLER_HPP