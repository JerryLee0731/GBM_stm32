#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PIDController {
public:
    PIDController(float kp, float ki, float kd);
    void setTunings(float kp, float ki, float kd);
    void setSetpoint(float setpoint);
    void setSampleTime(float sampleTime);
    void setOutputLimits(float min, float max);
    float compute(float input);

private:
    float kp;
    float ki;
    float kd;
    float setpoint;
    float sampleTime;
    float outputMin;
    float outputMax;

    float prevInput;
    float integral;
    float lastTime;
};

#endif // PIDCONTROLLER_H