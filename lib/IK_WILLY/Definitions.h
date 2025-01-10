#pragma once
#define _USE_MATH_DEFINES
#include <iostream>
#include <cmath>
#include <vector>
#include <stdexcept>
#include <string>

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <MPU6050_WE.h>

const double L1_TO_R1 = 126;
const double L1_TO_L3 = 167;
const double L2_TO_R2 = 163;
#define LEG_SITTING_Z -40.0


#define SERVOMIN 80  // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 480  // This is the 'maximum' pulse length count (out of 4096)
#define USMIN 600     // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX 2400    // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define PCA9685_ADDRESS 0x40
#define MPU6500_ADDRESS 0x68

#define WALK 0
#define ROTATE 1

#define WALK_FORWARD 0
#define WALK_BACKWARDS 1

#define ROTATE_LEFT 0
#define ROTATE_RIGHT 1



/*
# Length of leg segments in millimeters
COXA_LEN 43
FEMUR_LEN 60
TIBIA_LEN 104

# Distance between the coxa rotation centers of different legs in millimeters.
L1_TO_R1 126
L1_TO_L3 167
L2_TO_R2 163

# The height where the legs connect to the frame.
LEG_CONNECTION_Z -7
# The Z value for the leg endpoints when sitting on a flat surface.
LEG_SITTING_Z -40

# The angle between the servo itself and the leg segment when the servo is centered.
COXA_ATTACH_ANGLE -8
FEMUR_ATTACH_ANGLE 35
TIBIA_ATTACH_ANGLE 83
*/