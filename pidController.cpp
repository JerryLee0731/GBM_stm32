#include "pidController.h"
#include "mbed.h"

PIDController::PIDController(float kp, float ki, float kd)
    : kp(kp), ki(ki), kd(kd), setpoint(0), sampleTime(1.0), outputMin(0), outputMax(1),
      prevInput(0), integral(0), lastTime(0) {}

void PIDController::setTunings(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

void PIDController::setSetpoint(float setpoint) {
    this->setpoint = setpoint;
}

void PIDController::setSampleTime(float sampleTime) {
    this->sampleTime = sampleTime;
}

void PIDController::setOutputLimits(float min, float max) {
    if (min >= max) return;
    outputMin = min;
    outputMax = max;
}

float PIDController::compute(float input) {
    float now = us_ticker_read() / 1000000.0; // Convert to seconds
    float timeChange = (now - lastTime);
    
    if (timeChange >= sampleTime) {
        float error = setpoint - input;
        //printf("error = %d\r\n", int(error));
        integral += (ki * error * timeChange);
        if (integral > outputMax) integral = outputMax;
        else if (integral < outputMin) integral = outputMin;

        float dInput = (input - prevInput) / timeChange;

        float output = kp * error + integral - kd * dInput;
        //printf("output = %d\r\n", int(output * 1000));
        if (output > outputMax) output = outputMax;
        else if (output < outputMin) output = outputMin;
        //printf("output_after = %d\r\n", int(output * 1000));

        prevInput = input;
        lastTime = now;

        return output;
    }
    return 0;
}