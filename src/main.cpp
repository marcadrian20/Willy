#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "WillyIK.h"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
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

  Serial.begin(9600);
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
  if(Serial.available())//if reads 'w' then walk, 'r' rotate
  {
    char command = Serial.read();
    if(command == 'w')
    {
      quadruped.walkCrawl(stepLength, stepHeight, stepDuration,0);
    }
    else if(command == 'r')
    {
      quadruped.walkCrawl(stepLength, stepHeight, stepDuration,2);
    }
    else if(command == 'q')
    {
      quadruped.walkQuadruped(stepLength, stepHeight, stepDuration,0);
    }
    else if(command == 's')
    {
      quadruped.initializeStance();
    }
  }
  // quadruped.walkCrawl(stepLength, stepHeight, stepDuration);
  // quadruped.walkQuadruped(stepLength, stepHeight, stepDuration);
  // SpiderLeg spiderleg("1", 43, 60, 104);
  // std::cout<<"extend legs"<<std::endl;
  // auto angles = spiderleg.inverseKinematics({(L1_TO_R1 / 2)-20, (L1_TO_L3 / 2)-20, LEG_SITTING_Z});
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

void Hexapod::walkQuadruped(double stepLength, double stepHeight, double stepDuration,int motionType)
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
      auto target = calculateTrajectory(i, phase, stepLength, stepHeight, motionType);
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
      auto target = calculateTrajectory(i, phase, stepLength, stepHeight, motionType);
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

std::vector<double> Hexapod::calculateTrajectory(int legIndex, double phase, double stepLength, double stepHeight, int motionType)
{ // Neutral positions for the leg
  double defaultX = L1_TO_R1 / 2;
  double defaultY = L1_TO_L3 / 2;
  double defaultZ = LEG_SITTING_Z;

  // X is Horizontal motion (forward/backward)
  // Y is Lateral motion (side to side)
  // Z is Vertical motion (up and down)
  // Motion type 0 is forward/backward, 1 is side to side, 2 is rotation to left or right
  double x = defaultX, y = defaultY;
  // x = 0;
  std::cout << "Leg index" << legIndex << "Phase" << phase << std::endl;
  switch (motionType)
  {
  case 0:
    x = sin(M_PI * phase * 2) * stepLength;
    break;
  case 1:
    x = sin(M_PI * phase) * stepLength;
    y = x;
    break;
  case 2:
    if (legIndex % 2 == 0)
      x = sin(M_PI * phase * 2) * stepLength;
    else
      (x = -sin(M_PI * phase * 2) * stepLength);
    break;
  default:
    break;
  }
  std::cout << "X placement" << x << std::endl;
  // stepHeight * sin(M_PI * (1 - phase) * 2));       // Lower

  double z = defaultZ + ((phase <= 0.25) ? stepHeight * sin(M_PI * (1 - phase) * 4) : -stepHeight * cos(M_PI * phase * 1.35 + 0.465));
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
  bodyX = x; // #TODO: use forward kinematics to get leg pos
  bodyY = y; // then use it to compute adjusted angles based on the step data
  bodyZ = z; //

  // Update the position of each leg to account for the new body position
  for (int i = 0; i < legs.size(); ++i)
  {
    auto joints = quadruped.legs[i].forwardKinematics();
    // Calculate the default resting position for the leg
    double defaultX = joints[4][0]; //(i % 2 == 0) ? -L1_TO_R1 / 2 : L1_TO_R1 / 2; // Left legs negative, right legs positive
    double defaultY = joints[4][1];
    ; //(i < 3) ? L1_TO_L3 / 2 : -L1_TO_L3 / 2;      // Front legs positive, back legs negative
    double defaultZ = joints[4][2];
    ;

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
      Serial.print("Angles for leg ");
      Serial.print(i);
      Serial.print(": ");
      for (const auto &angle : angles)
      {
        Serial.print(angle);
        Serial.print(" ");
      }
      Serial.println();
      // setServoAngles(i, angles);
      // delay(50);
    }
  }
}

void Hexapod::walkCrawl(double stepLength, double stepHeight, double stepDuration,int motionType)
{
  // int legOrder[] = {0, 3, 1, 2}; // Front Left, Back Right, Front Right, Back Left
  // int legOrder[] = {0, 0, 0, 0};
  int legOrder[] = {0, 1, 2, 3};
  for (int legIndex : legOrder)
  {
    // Move one leg at a time
    for (double t = 0; t <= 1.0; t += 0.1)
    {
      double phase = t;
      auto target = calculateTrajectory(legIndex, phase, stepLength, stepHeight, motionType);
      auto angles = legs[legIndex].inverseKinematics(target);
      setServoAngles(legIndex, angles);
      // delay(10);
      // delay(delayTime * 1000);
    }

    // Shift the body slightly to balance the motion

    // setBodyPosition(bodyX + stepLength / 4, bodyY, bodyZ);
    // #TODO make set body pos work and fix gait generation
  }
}

void Hexapod::initializeStance()
{
  // Define neutral positions for each leg
  std::vector<std::vector<double>> neutralPositions = {
      {L1_TO_R1 / 2, L1_TO_L3 / 2, LEG_SITTING_Z}, // Front Left (L1)
      {L1_TO_R1 / 2, L1_TO_L3 / 2, LEG_SITTING_Z},   // Front Right (R1)
      {L1_TO_R1 / 2, L1_TO_L3 / 2, LEG_SITTING_Z}, // Rear Left (L3)
      {L1_TO_R1 / 2, L1_TO_L3 / 2, LEG_SITTING_Z},   // Rear Right (R3)
      {-L1_TO_R1 / 2, 0, LEG_SITTING_Z},             // Middle Left (L2)
      {L1_TO_R1 / 2, 0, LEG_SITTING_Z}               // Middle Right (R2)
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