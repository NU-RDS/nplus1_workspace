#include "pvPID.hpp"

PVPID::PVPID(float pos_kp, float pos_ki, float pos_kd,
             float vel_kp, float vel_ki, float vel_kd)
    : pos_kp_(pos_kp), pos_ki_(pos_ki), pos_kd_(pos_kd),
      vel_kp_(vel_kp), vel_ki_(vel_ki), vel_kd_(vel_kd),
      target_position_(0.0f), target_velocity_(0.0f),
      prev_pos_error_(0.0f), prev_vel_error_(0.0f),
      pos_integral_(0.0f), vel_integral_(0.0f),
      prev_time_(micros()),
      min_torque_(-1.0f), max_torque_(1.0f),
      pos_integral_limit_(1.0f), vel_integral_limit_(1.0f)
{
}

void PVPID::setTargets(float position, float velocity) {
    target_position_ = position;
    target_velocity_ = velocity;
}

float PVPID::update(float current_position, float current_velocity) {
    // Calculate time delta
    unsigned long current_time = micros();
    float dt = (current_time - prev_time_) / 1000000.0f;  // Convert to seconds
    prev_time_ = current_time;

    // Ensure dt is reasonable (handle micros() overflow or very long delays)
    if (dt > 0.1f || dt <= 0.0f) {
        dt = 0.001f;  // Default to 1ms if dt is unreasonable
    }

    // Position error calculations
    float pos_error = target_position_ - current_position;
    pos_integral_ += pos_error * dt;
    pos_integral_ = limit(pos_integral_, -pos_integral_limit_, pos_integral_limit_);
    float pos_derivative = (pos_error - prev_pos_error_) / dt;
    prev_pos_error_ = pos_error;

    // Position PID output (becomes velocity target)
    float velocity_target = target_velocity_ + 
                          pos_kp_ * pos_error +
                          pos_ki_ * pos_integral_ +
                          pos_kd_ * pos_derivative;

    // Velocity error calculations
    float vel_error = velocity_target - current_velocity;
    vel_integral_ += vel_error * dt;
    vel_integral_ = limit(vel_integral_, -vel_integral_limit_, vel_integral_limit_);
    float vel_derivative = (vel_error - prev_vel_error_) / dt;
    prev_vel_error_ = vel_error;

    // Calculate final torque command
    float torque = vel_kp_ * vel_error +
                  vel_ki_ * vel_integral_ +
                  vel_kd_ * vel_derivative;

    // Limit output torque
    return limit(torque, min_torque_, max_torque_);
}

void PVPID::reset() {
    prev_pos_error_ = 0.0f;
    prev_vel_error_ = 0.0f;
    pos_integral_ = 0.0f;
    vel_integral_ = 0.0f;
    prev_time_ = micros();
}

void PVPID::setPositionGains(float kp, float ki, float kd) {
    pos_kp_ = kp;
    pos_ki_ = ki;
    pos_kd_ = kd;
}

void PVPID::setVelocityGains(float kp, float ki, float kd) {
    vel_kp_ = kp;
    vel_ki_ = ki;
    vel_kd_ = kd;
}

void PVPID::setTorqueLimits(float min_torque, float max_torque) {
    min_torque_ = min_torque;
    max_torque_ = max_torque;
}

void PVPID::setIntegratorLimits(float pos_limit, float vel_limit) {
    pos_integral_limit_ = pos_limit;
    vel_integral_limit_ = vel_limit;
}

float PVPID::limit(float value, float min_val, float max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}