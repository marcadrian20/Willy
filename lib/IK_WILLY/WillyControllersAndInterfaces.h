#pragma once
#include "Definitions.h"


class ServoController {
private:
    Adafruit_PWMServoDriver pwm;
    int mapAngleToPulse(double angle);
public:
    ServoController(uint8_t address){
        pwm = Adafruit_PWMServoDriver(address);
    };
    void init();
    void setAngle(uint8_t servoNum, double angle);
    
};

class MPU6500Interface {
private:
    MPU6500_WE mpu6500;
public:
    MPU6500Interface(uint8_t address) : mpu6500(address) {};
    bool init();
    float getPitch();
    float getRoll();
};