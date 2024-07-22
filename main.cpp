#include "mbed.h"
#include <cmath>
#include "QEI.h"
#include "pidController.h"

double thetaf, thetar; // 轉向用馬達角度
double omegaf, omegar; // 前後輪走行用馬達轉速

float lastTime = 0;

int enc_vel_f_prev = 0;

PwmOut pwm_vel_f1(D14); // A0, 反轉 d12 d13待修正
PwmOut pwm_vel_f2(D15); // A1, 正轉 逆時針 enc正

PwmOut pwm_ang_f1(A0); // A0, 反轉
PwmOut pwm_ang_f2(A1); // A1, 正轉 逆時針 enc正

QEI encoder_vel_f(D6, D7, NC, 16);
QEI encoder_ang_f(D4, D5, NC, 16);

float kp_ang = 5.00/360;
float ki_ang = 0.02;
float kd_ang = 0.0001;
PIDController pid_angle(kp_ang, ki_ang, kd_ang);

float kp_vel = 1.00/900;
float ki_vel = 0.01;
float kd_vel = 0.00001;
PIDController pid_vel(kp_vel, ki_vel, kd_vel);

float MIN_PWM = 0.35; // 0.37
float MIN_VEL = 10;

AnalogIn a2(A2);

int enc2deg (int pulse) {
    return pulse * 360 / 1200;
}

float motor (float x) {
    float control = (1-MIN_PWM) * x + MIN_PWM;

    return control;
}

void vel2zero (float& vel) {
    if (vel > -MIN_VEL && vel < MIN_VEL) {
        vel = 0;
    }
}

void setMotorSpeed(float control, PwmOut& pwm_1, PwmOut& pwm_2) {
    if (control > 0) {
        pwm_1.write(0);
        if (control > 1) {
            control = 1;
        }
        pwm_2.write(motor(control));
    } // 0到1之間
    else if (control < 0) {
        pwm_2.write(0);
        if (control < -1) {
            control = -1;
        }
        pwm_1.write(motor(-control));
    }
}

float bound01 (float x) {
    if (x > 1) {
        x = 1;
    }
    if (x < -1) {
        x = 0;
    }
    return x;
}

float ff (float target) {
    if (target > 0) {
        float ans = (target + 313.19) / 1194.5; // from my test, recorded in excel
        return bound01(ans);
    }
    else if (target == 0) {
        return 0;
    }
    else {
        float ans = (-target + 313.19) / 1194.5;
        return -bound01(ans);
    }
}

// main() runs in its own thread in the OS
int main()
{
    // pc <-> stm32
    BufferedSerial serial_port(USBTX, USBRX, 115200);  // (tx,rx,baudrate)

    // 初始化motor pwm週期 10000Hz
    pwm_ang_f1.period(1.0 / 10000);
    pwm_ang_f2.period(1.0 / 10000);

    pwm_vel_f1.period(1.0 / 10000);
    pwm_vel_f2.period(1.0 / 10000);

    // 设置输出范围
    pid_angle.setOutputLimits(-0.8, 0.8);
    // 设置采样时间
    pid_angle.setSampleTime(0.01);

    pid_vel.setOutputLimits(-0.8, 0.8);
    pid_vel.setSampleTime(0.01);


    while (true) {
        
        float voltage = a2.read();  // 读取模拟信号，返回值范围在 0 到 1 之间
        //printf("voltage = %d\r\n", int(voltage * 1000));
        

        float now = us_ticker_read() / 1000000.0; // Convert to seconds
        float timeChange = (now - lastTime);
        lastTime = now;

        // motor
        // encoder
        int enc_ang_f = encoder_ang_f.getPulses(); // getRev 16: Pulse Per Rev // 1200 per rev
        int enc_vel_f = encoder_vel_f.getPulses(); 

        int ang_f = enc2deg(enc_ang_f);
        //printf("ang_f = %d\r\n", ang_f);

        float vel_f = enc2deg( (enc_vel_f - enc_vel_f_prev) / (timeChange) );
        //printf("vel_f = %d\r\n", int(vel_f));

        enc_vel_f_prev = enc_vel_f;
        // encoder end
        
        
        // motor control ang
        float target_ang = voltage * 1000;
        pid_angle.setSetpoint(target_ang);
        float control_ang_f = pid_angle.compute(ang_f);
        //printf("control_ang_f = %d\r\n", int(control_ang_f * 1000));
        setMotorSpeed(control_ang_f, pwm_ang_f1, pwm_ang_f2);

        
        // motor control vel, add feedfoward term
        float target_vel = voltage * 900 * 2 - 900;
        vel2zero(target_vel);
        //printf("target_vel = %d\r\n", int(target_vel));
        //printf("ff = %d\r\n", int(ff(target_vel) * 1000));
        pid_vel.setSetpoint(target_vel);
        float control_vel_f = pid_vel.compute(vel_f);
        control_vel_f = control_vel_f + ff(target_vel);
        //printf("control_vel_f = %d\r\n", int(control_vel_f * 1000));
        //setMotorSpeed(control_vel_f, pwm_vel_f1, pwm_vel_f2);
        
        // motor pwm end
        // motor end

        // serial port to pc
        //printf("%d,%d\n", int(target*1000), int(vel_f*1000));
        printf("%d,%d\n", int(target_ang*1000), int(ang_f*1000));

        ThisThread::sleep_for(10ms);
    }
}
