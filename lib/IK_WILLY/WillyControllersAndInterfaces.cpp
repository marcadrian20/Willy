#include "WillyControllersAndInterfaces.h"

void ServoController::init()
{
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ); // Analog servos run at ~50 Hz updates
    delay(10);
}

void ServoController::setAngle(uint8_t servoNum, double angle)
{
    int pulse = mapAngleToPulse(angle);
    pwm.setPWM(servoNum, 0, pulse);
}

int ServoController::mapAngleToPulse(double angle)
{
    int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
    return pulse;
}

bool MPU6500Interface::init()
{
    if (!mpu6500.init())
    {
        Serial.println("MPU6500 does not respond");
        return false;
    }
    Serial.println("MPU6500 is online");
    Serial.println("Position you MPU6500 flat and don't move it - calibrating...");
    delay(1000);
    mpu6500.autoOffsets();
    Serial.println("Done!");
    mpu6500.setSampleRateDivider(5);
    mpu6500.setAccRange(MPU6500_ACC_RANGE_2G);
    mpu6500.enableAccDLPF(true);
    mpu6500.setAccDLPF(MPU6500_DLPF_6);
    delay(1000);
    return true;
}

float MPU6500Interface::getPitch()
{
    return mpu6500.getPitch();
}

float MPU6500Interface::getRoll()
{
    return mpu6500.getRoll();
}


