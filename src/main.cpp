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

void InitializeWilly();
Hexapod quadruped(43, 60, 104);
void setServoAngles(int legIndex, const std::vector<double> &angles);
int coxa = 280;
int femur = 280;
int tibia = 280;
int invertcoxa = 280;
int invertfemur = 280;
int inverttibia = 280;
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
  //   auto angles = spiderleg.inverseKinematics({x, y, z});
  //   spiderleg.printJointPositions();
  //   angles[0] = max(-90.0, min(angles[0], 90.0));
  //   angles[1] = max(-90.0, min(angles[1], 90.0));
  //   angles[2] = max(-90.0, min(angles[2], 90.0));

  //   coxa = map(angles[0] + 8, -90, 90, 80, 480);
  //   femur = map(angles[1] - 35, -90, 90, 80, 480);
  //   tibia = map(angles[2] - 83, -90, 90, 80, 480);

  //   // pwm.setPWM(6, 0, coxa);
  //   // pwm.setPWM(7, 0, femur);
  //   // pwm.setPWM(8, 0, tibia);
  //   pwm.setPWM(9, 0, coxa);
  //   pwm.setPWM(10, 0, femur);
  //   pwm.setPWM(11, 0, tibia);
  //   // coxa = 480 + 80 - coxa;
  //   // femur = 480 + 80 - femur;
  //   // tibia = 480 + 80 - tibia;
  //   pwm.setPWM(0, 0, coxa);
  //   pwm.setPWM(1, 0, femur);
  //   pwm.setPWM(2, 0, tibia);
  //   // pwm.setPWM(3, 0, coxa);
  //   // pwm.setPWM(4, 0, femur);
  //   // pwm.setPWM(5, 0, tibia);
  // }
  // quadruped.initializeStance();
  quadruped.walkCrawl(stepLength, stepHeight, stepDuration);
  // SpiderLeg spiderleg("1",43,60,104);
  // auto angles = spiderleg.inverseKinematics({0, 0, 0});
  // setServoAngles(0, angles);
  // quadruped.setBodyPosition(10, 0, LEG_SITTING_Z + 20);
  delay(1000);
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

void Hexapod::walkQuadruped(double stepLength, double stepHeight, double stepDuration)
{
  int group1[] = {0, 3}; //{0, 5}; // Front Left (Leg 1) and Back Right (Leg 6)
  int group2[] = {1, 2}; //{1, 4}; // Front Right (Leg 2) and Back Left (Leg 5)

  double timeStep = 0.01; // Small time increment for smooth stepping
  for (double t = 0; t < stepDuration; t += timeStep)
  {
    double phase = t / stepDuration; // Phase progresses from 0 to 1

    // Move group 1 during the first half of the step
    if (phase < 0.5)
    {
      for (int i : group1)
      {
        auto target = calculateTrajectory(i, phase, stepLength, stepHeight);
        auto angles = legs[i].inverseKinematics(target);
        setServoAngles(i, angles);
        delay(100);
        // moveLeg(i, target);
      }
    }

    // Move group 2 during the second half of the step
    if (phase >= 0.5)
    {
      for (int i : group2)
      {
        auto target = calculateTrajectory(i, phase, stepLength, stepHeight);
        auto angles = legs[i].inverseKinematics(target);
        setServoAngles(i, angles);
        delay(100);
        // moveLeg(i, target);
      }
    }

    // Synchronize body shift with the gait phase
    if (phase == 0.25 || phase == 0.75)
    {
      setBodyPosition(bodyX + stepLength / 2, bodyY, bodyZ);
    }
  }

  // Finalize body position after a full step
  setBodyPosition(bodyX + stepLength, bodyY, bodyZ);
}

std::vector<double> Hexapod::calculateTrajectory(int legIndex, double phase, double stepLength, double stepHeight)
{                      // Neutral positions for the leg
  double defaultX = 0; //(legIndex % 2 != 0) ? -L1_TO_R1 / 2 : L1_TO_R1 / 2; // Left legs negative, right legs positive
  double defaultY = 0; //(legIndex < 2) ? L1_TO_L3 / 2 : -L1_TO_L3 / 2;     // Front legs positive, back legs negative
  double defaultZ = LEG_SITTING_Z;

  // Horizontal motion (forward/backward)
  double x;
  if (phase <= 0.5)
  {                                              // Forward swing
    x = defaultX + (2 * phase - 1) * stepLength; // Forward motion during the swing
  }
  else
  {                                                          // Backward placement
    x = defaultX - (2 * (phase - 0.5) - 1) * stepLength / 2; // Backward motion during placement
  }
  // ##TODO: FIX everything. It has the traditional shape of the curve but
  // calculations are wrong. Either fix sinusoidal curves or use bezier curves
  // Fix atachment points and the forward swing, Lift,swing,lower, pull,shift body,continue
  // i presume that im here:     ^^        and i should be here:   ^^
  //                           ^    ^                            ^    ^
  //                          ^^^^^^^^ <-      direction^     ->^^^^^^^^
  //                   direction  <-
  // // Vertical motion (lift/lower)
  // double z = defaultZ + ((phase > 0.5) ? 0 : stepHeight); // stepHeight * sin(M_PI * phase * 2)         // Lift
  //: stepHeight * sin(M_PI * (1 - phase) * 2)); // Lower
  double z = defaultZ + ((phase > 0.5) ? 0 : stepHeight * sin(M_PI * phase * 2));
  // ////////////!!!!!!!!!!!!!!!!!
  // double z = defaultZ + ((phase > 0.5) ?stepHeight * sin(M_PI * phase * 2)         // Lift
  //   : stepHeight * sin(M_PI * (1 - phase) * 2)); // Lower
  // walks better but doesnt raise foot
  // ///////////!!!!!!!!!!!!!!!!!!
  // // Lateral motion (fixed for crawling)
  // double y = defaultY + 20 * cos(M_PI * phase);
  // double x = defaultX + phase * stepLength;                 // defaultX + stepLength * 0.5 * (1 - cos(2 * M_PI * phase));                                     // Linear horizontal motion
  // double z = defaultZ + stepHeight * sin(2 * M_PI * phase); // Sinusoidal vertical lift
  double y = defaultY; // +15 * cos(M_PI * phase);
  return {x, y, z};
}

void setServoAngles(int legIndex, const std::vector<double> &angles)
{
  std::cout<<"Leg index"<<legIndex<<std::endl;
  // Map joint angles to servo pulses
  if (legIndex == 4 || legIndex == 5)
    return;
  auto coxaAng = max(-180.0, min(angles[0] + 8, 180.0));
  auto femurAng = max(-180.0, min(angles[1] - 35, 180.0));
  auto tibiaAng = max(-180.0, min(angles[2] - 83, 180.0));
  if (legIndex % 2 != 0)
  {
    coxaAng = -coxaAng;   // #TODO find better way to mirror angles
    femurAng = -femurAng; // for the right/left side
    tibiaAng = -tibiaAng; //
  }
  coxa = map(coxaAng, -180, 180, 80, 480);
  femur = map(femurAng, -180, 180, 80, 480);
  tibia = map(tibiaAng, -180, 180, 80, 480);
  // Set servos for the given leg
  pwm.setPWM(legIndex * 3 + 1, 0, femur); // Femur servo
  std::cout<<coxa<<" "<<femur<<" "<<tibia<<std::endl;
  pwm.setPWM(legIndex * 3 + 2, 0, tibia); // Tibia servo
  std::cout<<coxa<<" "<<femur<<" "<<tibia<<std::endl;
  pwm.setPWM(legIndex * 3 + 0, 0, coxa);  // Coxa servo
  std::cout<<coxa<<" "<<femur<<" "<<tibia<<std::endl;
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
    double defaultX = 0; //(i % 2 == 0) ? -L1_TO_R1 / 2 : L1_TO_R1 / 2; // Left legs negative, right legs positive
    double defaultY = 0; //(i < 3) ? L1_TO_L3 / 2 : -L1_TO_L3 / 2;      // Front legs positive, back legs negative
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
      Serial.print("Angles for leg ");
      Serial.print(i);
      Serial.print(": ");
      for (const auto &angle : angles)
      {
        Serial.print(angle);
        Serial.print(" ");
      }
      Serial.println();
      setServoAngles(i, angles);
      delay(50);
    }
  }
}

void Hexapod::walkCrawl(double stepLength, double stepHeight, double stepDuration)
{
  int legOrder[] = {0, 3, 1, 2}; // Front Left, Back Right, Front Right, Back Left
                                 // double delayTime = stepDuration / 20;
  for (int legIndex : legOrder)
  {
    // Move one leg at a time
    for (double t = 0; t <= 1.0; t += 0.05)
    {
      double phase = t;
      auto target = calculateTrajectory(legIndex, phase, stepLength, stepHeight);
      auto angles = legs[legIndex].inverseKinematics(target);
      setServoAngles(legIndex, angles);
      delay(50);
      // delay(delayTime * 1000);
    }

    // Shift the body slightly to balance the motion

    setBodyPosition(bodyX + stepLength / 4, bodyY, bodyZ);
    // #TODO make set body pos work and fix gait generation
  }
}

void Hexapod::initializeStance()
{
  // Define neutral positions for each leg
  std::vector<std::vector<double>> neutralPositions = {
      {-L1_TO_R1 / 2, -L1_TO_L3 / 2, LEG_SITTING_Z}, // Front Left (L1)
      {L1_TO_R1 / 2, L1_TO_L3 / 2, LEG_SITTING_Z},   // Front Right (R1)
      {-L1_TO_R1 / 2, -L1_TO_L3 / 2, LEG_SITTING_Z}, // Rear Left (L3)
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
