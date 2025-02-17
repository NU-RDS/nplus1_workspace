// #include <vector>
// #include <cmath>
// #include <chrono>

// #include "pvPID.hpp"

// class JointPIDController {
// private:
//     // Gains
//     double kp, ki, kd;
    
//     // Error terms
//     double integral_error;
//     double prev_error;
    
//     // Control limits
//     double max_torque;
//     double min_torque;
    
//     // Timing
//     std::chrono::steady_clock::time_point last_update;
    
//     // Initialize timing
//     void initTiming() {
//         last_update = std::chrono::steady_clock::now();
//     }
    
// public:
//     JointPIDController(double p_gain, double i_gain, double d_gain, 
//                       double max_t, double min_t) 
//         : kp(p_gain), ki(i_gain), kd(d_gain),
//           max_torque(max_t), min_torque(min_t),
//           integral_error(0.0), prev_error(0.0) {
//         initTiming();
//     }
    
//     // Reset controller state
//     void reset() {
//         integral_error = 0.0;
//         prev_error = 0.0;
//         initTiming();
//     }
    
//     // Compute control output
//     double computeTorque(double target_angle, double current_angle, double feedforward = 0.0) {
//         // Get time delta
//         auto current_time = std::chrono::steady_clock::now();
//         double dt = std::chrono::duration<double>(current_time - last_update).count();
//         last_update = current_time;
        
//         // Compute error
//         double error = target_angle - current_angle;
        
//         // Update integral term with anti-windup
//         integral_error += error * dt;
        
//         // Compute derivative term
//         double derivative = dt > 0.0 ? (error - prev_error) / dt : 0.0;
//         prev_error = error;
        
//         // Compute PID output
//         double output = kp * error + ki * integral_error + kd * derivative;
        
//         // Add feedforward term
//         output += feedforward;
        
//         // Apply torque limits
//         return std::max(min_torque, std::min(max_torque, output));
//     }
// };

// class FingerController {
// private:
//     JointPIDController proximal_joint;
//     JointPIDController distal_joint;
    
// public:
//     FingerController(
//         double prox_p, double prox_i, double prox_d,
//         double dist_p, double dist_i, double dist_d,
//         double max_torque = 1.0, double min_torque = -1.0)
//         : proximal_joint(prox_p, prox_i, prox_d, max_torque, min_torque),
//           distal_joint(dist_p, dist_i, dist_d, max_torque, min_torque) {}
    
//     std::vector<double> computeTorques(
//         const std::vector<double>& target_angles,
//         const std::vector<double>& current_angles,
//         const std::vector<double>& feedforward = {0.0, 0.0}) {
        
//         if (target_angles.size() != 2 || current_angles.size() != 2) {
//             throw std::invalid_argument("Both target and current angles must have size 2");
//         }
        
//         std::vector<double> torques(2);
//         torques[0] = proximal_joint.computeTorque(
//             target_angles[0], current_angles[0], 
//             feedforward.size() > 0 ? feedforward[0] : 0.0);
        
//         torques[1] = distal_joint.computeTorque(
//             target_angles[1], current_angles[1],
//             feedforward.size() > 1 ? feedforward[1] : 0.0);
        
//         return torques;
//     }
    
//     void reset() {
//         proximal_joint.reset();
//         distal_joint.reset();
//     }
// };

// // Example usage
// int main() {
//     // Initialize controller with PID gains
//     FingerController finger(
//         1.0, 0.1, 0.05,  // Proximal joint PID gains
//         0.8, 0.08, 0.04, // Distal joint PID gains
//         2.0,  // Max torque
//         -2.0  // Min torque
//     );
    
//     // Target angles (in radians)
//     std::vector<double> targets = {M_PI/4, M_PI/3};
    
//     // Current angles (in radians)
//     std::vector<double> current = {0.0, 0.0};
    
//     // Optional feedforward terms
//     std::vector<double> feedforward = {0.1, 0.1};
    
//     // Compute control torques
//     std::vector<double> torques = finger.computeTorques(targets, current, feedforward);
    
//     return 0;
// }