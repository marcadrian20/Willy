#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "WillyIK.h"
#include <MPU6050_WE.h>
#include <Wire.h>

#define MPU6500_ADDRESS 0x68

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
MPU6500_WE mpu6500 = MPU6500_WE(MPU6500_ADDRESS);

#define SERVOMIN 150  // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 600  // This is the 'maximum' pulse length count (out of 4096)
#define USMIN 600     // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX 2400    // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

Hexapod quadruped(43, 60, 104);
void setServoAngles(int legIndex, const std::vector<double> &angles);

double stepLength = 50;
double stepHeight = 20;
double stepDuration = 1.0;
// SpiderLeg spiderleg("LEG1", 43, 60, 104);
// std::vector<std::vector<int>> ServoMap = {{9, 10, 11}, {0, 1, 2}, {12, 12, 14}, {15, 16, 17}, {6, 7, 8}, {3, 4, 5}};
void setup()
{
  // quadruped.setBodyPosition(0, 0, -40);

  Serial.begin(115200);
  Wire.begin();

  if (!mpu6500.init())
  {
    Serial.println("MPU6500 does not respond");
  }
  else
  {
    Serial.println("MPU6500 is connected");
  }
  Serial.println("Position you MPU6500 flat and don't move it - calibrating...");
  delay(1000);
  mpu6500.autoOffsets();
  Serial.println("Done!");
  
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ); // Analog servos run at ~50 Hz updates

  // InitializeWilly();
  delay(10);
}

void loop()
{

  // if (Serial.available())
  // {
  //   String data = Serial.readStringUntil('\n');
  //   float x = data.substring(0, data.indexOf(',')).toFloat();
  //   float y = data.substring(data.indexOf(',') + 1, data.lastIndexOf(',')).toFloat();
  //   float z = data.substring(data.lastIndexOf(',') + 1).toFloat();
  //   Serial.println("X: " + String(x) + " Y: " + String(y) + " Z: " + String(z));
  // }
  // quadruped.initializeStance();
  if (Serial.available()) // if reads 'w' then walk, 'r' rotate
  {
    char command = Serial.read();
    if (command == 'w')
    {
      quadruped.walkWaveGait(stepLength, stepHeight, stepDuration, 0, 0);
    }
    else if (command == 's')
    {
      quadruped.walkWaveGait(stepLength, stepHeight, stepDuration, 0, 1);
    }
    else if (command == 'a')
    {
      quadruped.walkWaveGait(stepLength, stepHeight, stepDuration, 2, 0);
    }
    else if (command == 'd')
    {
      quadruped.walkWaveGait(stepLength, stepHeight, stepDuration, 2, 1);
    }
    else if (command == 'q')
    {
      quadruped.walkQuadruped(stepLength, stepHeight, stepDuration, 0, 0);
    }
  }
  // quadruped.walkCrawl(stepLength, stepHeight, stepDuration);
  // quadruped.walkQuadruped(stepLength, stepHeight, stepDuration);
  // SpiderLeg spiderleg("1", 43, 60, 104);
  // // std::cout<<"extend legs"<<std::endl;
  // auto angles = spiderleg.inverseKinematics({0, (L1_TO_L3 / 2), LEG_SITTING_Z+10});
  // setServoAngles(0, angles);
  // setServoAngles(1, angles);
  // setServoAngles(2, angles);
  // setServoAngles(3, angles);
  // delay(1000);

  // std::cout<<"sit legs"<<std::endl;
  //  angles = spiderleg.inverseKinematics({(L1_TO_R1 / 2) -20, (L1_TO_L3 / 2) -20, LEG_SITTING_Z});
  // setServoAngles(0, angles);
  // setServoAngles(1, angles);
  // setServoAngles(2, angles);
  // setServoAngles(3, angles);
  // delay(1000);
  // std::cout<<"pull in legs"<<std::endl;
  // angles = spiderleg.inverseKinematics({(L1_TO_R1 / 2) +20, (L1_TO_L3 / 2) +20, LEG_SITTING_Z});
  // setServoAngles(0, angles);
  // setServoAngles(1, angles);
  // setServoAngles(2, angles);
  // setServoAngles(3, angles);

  // delay(1000);
}
void InitializeWilly()
{
  int pulselen = 280;
  pwm.setPWM(0, 0, pulselen);
  pwm.setPWM(1, 0, pulselen);
  pwm.setPWM(2, 0, pulselen);
  pwm.setPWM(3, 0, pulselen);
  pwm.setPWM(4, 0, pulselen);
  pwm.setPWM(5, 0, pulselen);
  pwm.setPWM(6, 0, pulselen);
  pwm.setPWM(7, 0, pulselen);
  pwm.setPWM(8, 0, pulselen);
  pwm.setPWM(9, 0, pulselen);
  pwm.setPWM(10, 0, pulselen);
  pwm.setPWM(11, 0, pulselen);
  delay(100);
}

void Hexapod::walkQuadruped(double stepLength, double stepHeight, double stepDuration, int motionType, int direction)
{
  int group1[] = {0, 3}; //{0, 5}; // Front Left (Leg 1) and Back Right (Leg 6)
  int group2[] = {1, 2}; //{1, 4}; // Front Right (Leg 2) and Back Left (Leg 5)
  // int group1[] = {0, 2};
  // int group2[] = {1, 3};
  double timeStep = 0.1; // Small time increment for smooth stepping
  for (double t = 0; t < stepDuration; t += timeStep)
  {
    double phase = t; // Phase progresses from 0 to 1

    // Move group 1 during the first half of the step
    for (int i : group1)
    {
      auto target = calculateTrajectory(i, phase, stepLength, stepHeight, motionType, direction);
      auto angles = legs[i].inverseKinematics(target);
      setServoAngles(i, angles);
      delay(10);
      // moveLeg(i, target);
    }
  }
  for (double t = 0; t < stepDuration; t += timeStep)
  {
    double phase = t; // Phase progresses from 0 to 1
    // Move group 2 during the second half of the step
    for (int i : group2)
    {
      auto target = calculateTrajectory(i, phase, stepLength, stepHeight, motionType, direction);
      auto angles = legs[i].inverseKinematics(target);
      setServoAngles(i, angles);
      delay(10);
      // moveLeg(i, target);
    }
  }
  // Synchronize body shift with the gait phase
  // if (phase == 0.25 || phase == 0.75)
  // {
  //   setBodyPosition(bodyX + stepLength / 2, bodyY, bodyZ);
  // }

  // Finalize body position after a full step
  // setBodyPosition(bodyX + stepLength, bodyY, bodyZ);
}

void adjustPosture()
{
  // Adjust the posture of the robot to ensure stability
  // This function can be used to correct the robot's posture
  // after a step or during a gait cycle.
}

std::vector<double> Hexapod::calculateTrajectory(int legIndex, double phase, double stepLength, double stepHeight, int motionType, int direction)
{ // Neutral positions for the leg
  double defaultX = L1_TO_R1 / 2;
  double defaultY = L1_TO_L3 / 2;
  double defaultZ = LEG_SITTING_Z;
  // Direction 0 is forward, 1 is backward, 2 is left, 3 is right
  //  X is Horizontal motion (forward/backward)
  //  Y is Lateral motion (side to side)
  //  Z is Vertical motion (up and down)
  //  Motion type 0 is forward/backward, 1 is side to side,if motion type is 2, 0 is left, 1 is right
  double x = defaultX, y = defaultY;
  // x = 0;
  std::cout << "Leg index" << legIndex << "Phase" << phase << std::endl;
  double z = defaultZ + ((phase <= 0.25) ? stepHeight * sin(M_PI * (1 - phase) * 4) : (phase > 0.70) ? (stepHeight / 2) * sin(M_PI * phase * 4)
                                                                                                     : -(stepHeight / 2) * cos(M_PI * phase * 1.35 + 0.465));
  switch (motionType)
  {
  case 0:
    std::cout << "Motion type 0 Direction" << direction << std::endl;
    x = (!direction) ? sin(M_PI * phase * 2) * stepLength : -sin(M_PI * phase * 2) * stepLength;
    break;
  case 1:
    // x = sin(M_PI * phase) * stepLength;
    // y = x;
    z = defaultZ;
    break;
  case 2:
    x = (direction) ? sin(M_PI * phase * 2) * stepLength : -sin(M_PI * phase * 2) * stepLength;
    x = (legIndex % 2 != 0) ? -x : x;
    break;
  default:
    break;
  }
  std::cout << "X placement" << x << std::endl;
  // stepHeight * sin(M_PI * (1 - phase) * 2));       // Lower

  std::cout << "Z placement" << z << std::endl;
  // Leg_sitting_Z(-40) is the default height of the leg, if you go for example -20, the bot will go up,basically pushes the leg down
  // double y = defaultY; // defaultY + ((phase <= 0.5) ? stepLength * sin(M_PI * phase) : -sin(M_PI*(1-phase))*stepLength);// + stepLength * sin(M_PI * phase); // Sinusoidal lateral motion

  std::cout << "Y placement" << y << std::endl;
  return {x, y, z};
  // Improve the calculateTrajectory function by:
  // 1. Fixing the sinusoidal curves for smoother leg movement.
  // 2. Ensuring the leg lifts properly during the swing phase.
  // 3. Adjusting the attachment points and forward swing for better gait.
  // 4. Considering the use of Bezier curves for more natural motion.
}

void setServoAngles(int legIndex, const std::vector<double> &angles)
{
  // std::cout << "Leg index" << legIndex << std::endl;
  // Map joint angles to servo pulses
  if (legIndex == 4 || legIndex == 5)
    return;
  std::cout << angles[0] << " " << angles[1] << " " << angles[2] << std::endl;
  auto coxaAng = max(0.0, min(angles[0] + 8, 180.0));
  auto femurAng = max(0.0, min(angles[1] - 35, 180.0));
  auto tibiaAng = max(0.0, min(angles[2] - 83, 180.0));
  if (legIndex % 2 == 0)
  {
    tibiaAng = 180 - tibiaAng;
    femurAng = 180 - femurAng;
    coxaAng = 180 - coxaAng;
  }
  int coxa = map(coxaAng, 0, 180, 80, 480);
  int femur = map(femurAng, 0, 180, 80, 480);
  int tibia = map(tibiaAng, 0, 180, 80, 480);
  // Set servos for the given leg
  pwm.setPWM(legIndex * 3 + 1, 0, femur); // Femur servo
  // std::cout << coxa << " " << femur << " " << tibia << std::endl;
  pwm.setPWM(legIndex * 3 + 2, 0, tibia); // Tibia servo
  // std::cout << coxa << " " << femur << " " << tibia << std::endl;
  pwm.setPWM(legIndex * 3 + 0, 0, coxa); // Coxa servo
  // std::cout << coxa << " " << femur << " " << tibia << std::endl;
  std::cout << "^^^^^^^^^^^^^^^^" << std::endl;
}
void Hexapod::setBodyPosition(double x, double y, double z)
{
  bodyX = x;
  bodyY = y;
  bodyZ = z; //

  // Update the position of each leg to account for the new body position
  for (int i = 0; i < legs.size(); ++i)
  {
    // Default position of the leg
    double defaultX = 0; // L1_TO_R1 / 2;
    double defaultY = L1_TO_L3 / 2;
    double defaultZ = LEG_SITTING_Z;
    // Adjust the leg's position based on the body's new position
    std::vector<double> targetPosition = {
        defaultX + bodyX, // Adjust forward/backward
        defaultY + bodyY, // Adjust lateral position
        defaultZ + bodyZ  // Adjust height
    };

    // Move the leg to the updated position
    if (i != 5 && i != 4)
    {
      auto angles = legs[i].inverseKinematics(targetPosition);
      setServoAngles(i, angles);
      delay(50);
    }
  }
}

void Hexapod::walkCrawl(double stepLength, double stepHeight, double stepDuration, int motionType, int direction)
{
  // int legOrder[] = {0, 3, 1, 2}; // Front Left, Back Right, Front Right, Back Left
  int legOrder[] = {0, 1, 2, 3};
  for (int legIndex : legOrder)
  {
    // Move one leg at a time
    for (double t = 0; t <= 1.01; t += 0.1)
    {
      double phase = t;
      auto target = calculateTrajectory(legIndex, phase, stepLength, stepHeight, motionType, direction);
      auto angles = legs[legIndex].inverseKinematics(target);
      setServoAngles(legIndex, angles);
      delay(50);
    }
  }
  // double offsetX = (direction) ? 50 : -50;
  // for (double t = 0; t <= 1.01; t += 0.1)
  // {
  //   // setBodyPosition(direction * sin(M_PI * (1 - t)), 0, sin(M_PI * t));

  //   delay(10);
  // }
}
// setBodyPosition(stepLength * sin(M_PI * 2 * phase), 0, stepHeight * sin(M_PI * phase * 4));
// delay(20);
// setBodyPosition(0,0,5);
// delay(20);
// setBodyPosition(0,0,0);
// // Shift the body slightly to balance the motion

std::vector<double> Hexapod::calculateWaveTrajectory(int legIndex, double phase, double stepLength, double stepHeight, int motionType, int direction)
{
  double defaultX = L1_TO_R1 / 2;
  double defaultY = L1_TO_L3 / 2;
  double defaultZ = LEG_SITTING_Z;
  // x(t)=x0+L*phase where x0 is the ini position , L is steplength and phase is the time
  double x = defaultX;
  double z = defaultZ;
  double y = defaultY;
  if (phase <= 0.25)
  {
    x = defaultX + stepLength * phase;
    z = defaultZ - stepHeight * sin(M_PI * phase);
    // using a sinusoidal curve: Z(t)=Z0+H*sin(pi*phase) where Z0 is the initial height, H is the step height and phase is the time
    // Y(t)=y(0) constant
  }
  else if (phase > 0.25 && phase <= 0.5)
  {
    x = defaultX + stepLength * (0.5 - phase);
    z = defaultZ - stepHeight * sin(M_PI * phase);
  }
  else if (phase > 0.5 && phase <= 0.75)
  {
    x = defaultX - stepLength * (phase - 0.5);
    z = defaultZ - stepHeight * sin(M_PI * phase);
  }
  else
  {
    x = defaultX - stepLength * (1 - phase);
    z = defaultZ - stepHeight * sin(M_PI * phase);
  }

  std::cout << "Leg index" << legIndex << "Phase" << phase << std::endl;
  std::cout << "X placement" << x << std::endl;
  std::cout << "Z placement" << z << std::endl;
  std::cout << "Y placement" << y << std::endl;
  return {x, y, z};
}
void Hexapod::walkWaveGait(double stepLength, double stepHeight, double stepDuration, int motionType, int direction)
{
  // Phase offsets for each leg (0.25 phase difference)
  double legPhases[4] = {0.0, 0.25, 0.5, 0.75};

  // Single gait cycle
  for (double t = 0; t <= 1.0; t += 0.05)
  {
    // Move all legs according to their phases
    for (int i = 0; i < 4; i++)
    {
      double legPhase = t + legPhases[i];
      while (legPhase >= 1.0)
        legPhase -= 1.0;

      auto target = calculateTrajectory(i, legPhase, stepLength, stepHeight, motionType, direction);
      auto angles = legs[i].inverseKinematics(target);
      setServoAngles(i, angles);
    }
    delay(stepDuration * 50); // Adjust timing as needed
  }
}

void Hexapod::initializeStance()
{
  // Define neutral positions for each leg
  std::vector<std::vector<double>> neutralPositions = {
      {L1_TO_R1 / 2, L1_TO_L3 / 2, LEG_SITTING_Z}, // Front Left (L1)
      {L1_TO_R1 / 2, L1_TO_L3 / 2, LEG_SITTING_Z}, // Front Right (R1)
      {L1_TO_R1 / 2, L1_TO_L3 / 2, LEG_SITTING_Z}, // Rear Left (L3)
      {L1_TO_R1 / 2, L1_TO_L3 / 2, LEG_SITTING_Z}, // Rear Right (R3)
      {-L1_TO_R1 / 2, 0, LEG_SITTING_Z},           // Middle Left (L2)
      {L1_TO_R1 / 2, 0, LEG_SITTING_Z}             // Middle Right (R2)
  };

  // Apply IK to move each leg to its neutral position
  for (size_t i = 0; i < legs.size(); ++i)
  {
    auto angles = legs[i].inverseKinematics(neutralPositions[i]);
    setServoAngles(i, angles); // Send angles to the servos
  }
}
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