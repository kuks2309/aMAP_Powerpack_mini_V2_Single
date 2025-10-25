#include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd)
    : kp(kp), ki(ki), kd(kd), error_sum(0.0), error_old(0.0),
      output_min(-1000.0), output_max(1000.0),
      integral_limit_enabled(false), integral_limit(100.0)
{
}

void PIDController::setGains(float kp, float ki, float kd)
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

void PIDController::setOutputLimits(float min_output, float max_output)
{
    this->output_min = min_output;
    this->output_max = max_output;
}

void PIDController::setIntegralLimit(float limit)
{
    this->integral_limit = limit;
    this->integral_limit_enabled = true;
}

void PIDController::reset()
{
    error_sum = 0.0;
    error_old = 0.0;
}

float PIDController::compute(float setpoint, float input)
{
    float error = setpoint - input;
    return compute(error);
}

float PIDController::compute(float error)
{
    // Integral term
    error_sum += error;

    // Integral windup prevention
    if (integral_limit_enabled)
    {
        if (error_sum > integral_limit)
            error_sum = integral_limit;
        if (error_sum < -integral_limit)
            error_sum = -integral_limit;
    }

    // Derivative term
    float error_d = error - error_old;

    // PID calculation
    float output = kp * error + ki * error_sum + kd * error_d;

    // Output limiting
    if (output > output_max)
        output = output_max;
    if (output < output_min)
        output = output_min;

    // Store for next iteration
    error_old = error;

    return output;
}
