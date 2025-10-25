#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController
{
private:
    float kp, ki, kd;
    float error_sum;
    float error_old;
    float output_min, output_max;
    bool integral_limit_enabled;
    float integral_limit;

public:
    PIDController(float kp, float ki, float kd);

    void setGains(float kp, float ki, float kd);
    void setOutputLimits(float min_output, float max_output);
    void setIntegralLimit(float limit);
    void reset();

    float compute(float setpoint, float input);
    float compute(float error); // 오버로드: 에러가 이미 계산된 경우

    // getter functions
    float getKp() const
    {
        return kp;
    }
    float getKi() const
    {
        return ki;
    }
    float getKd() const
    {
        return kd;
    }
    float getErrorSum() const
    {
        return error_sum;
    }
    float getErrorOld() const
    {
        return error_old;
    }
};

#endif
