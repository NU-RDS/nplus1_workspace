#ifndef FINGER_PID_CONTROLLER_HPP
#define FINGER_PID_CONTROLLER_HPP

#include <vector>
#include <chrono>
#include <stdexcept>
#include <cmath>

/**
 * @brief PID controller for a single joint
 * 
 * Implements a PID control loop with anti-windup protection,
 * torque limiting, and feedforward support.
 */
class JointPIDController {
private:
    // PID gains
    double kp;  ///< Proportional gain
    double ki;  ///< Integral gain
    double kd;  ///< Derivative gain
    
    // Error terms
    double integral_error;  ///< Accumulated integral error
    double prev_error;     ///< Previous error for derivative calculation
    
    // Control limits
    double max_torque;  ///< Maximum allowable torque output
    double min_torque;  ///< Minimum allowable torque output
    
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
    JointPIDController(double p_gain, double i_gain, double d_gain, 
                      double max_t, double min_t);
    
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
    double computeTorque(double target_angle, double current_angle, 
                        double feedforward = 0.0);
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
        double prox_p, double prox_i, double prox_d,
        double dist_p, double dist_i, double dist_d,
        double max_torque = 1.0, double min_torque = -1.0);
    
    /**
     * @brief Compute control torques for both joints
     * 
     * @param target_angles Vector of target angles [proximal, distal] in radians
     * @param current_angles Vector of current angles [proximal, distal] in radians
     * @param feedforward Vector of feedforward terms (optional)
     * @return std::vector<double> Computed torques for both joints
     * @throws std::invalid_argument if input vectors are not size 2
     */
    std::vector<double> computeTorques(
        const std::vector<double>& target_angles,
        const std::vector<double>& current_angles,
        const std::vector<double>& feedforward = {0.0, 0.0});
    
    /**
     * @brief Reset both joint controllers
     */
    void reset();
};

#endif // FINGER_PID_CONTROLLER_HPP