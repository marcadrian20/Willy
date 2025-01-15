#pragma once
#include "Definitions.h"

class Hexapod;
class ServoController
{
private:
    Adafruit_PWMServoDriver pwm;
    int mapAngleToPulse(double angle);

public:
    ServoController(uint8_t address)
    {
        pwm = Adafruit_PWMServoDriver(address);
    };
    void init();
    void setAngle(uint8_t servoNum, double angle);
};

class MPU6500Interface
{
private:
    MPU6500_WE mpu6500;

public:
    MPU6500Interface(uint8_t address) : mpu6500(address) {};
    bool init();
    float getPitch();
    float getRoll();
};

class PIDController
{
private:
    float kp, ki, kd;
    float previousError;
    float integral;
    float setpoint;
    unsigned long lastTime;

public:
    PIDController(float p, float i, float d, float target)
        : kp(p), ki(i), kd(d), setpoint(target), previousError(0), integral(0), lastTime(0) {}

    float calculate(float input);
    void reset();
    void setTarget(float target);
};

class BalanceController
{
private:
    PIDController pitchPID;
    PIDController rollPID;
    Hexapod &robot;

public:
    BalanceController(Hexapod &hexapod);
    void balance(float pitch, float roll);
    void reset();
};