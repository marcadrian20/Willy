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
double stepLength = 140;
double stepHeight = 30;
double stepDuration = 1.0;
// SpiderLeg spiderleg("LEG1", 43, 60, 104);
// std::vector<std::vector<int>> ServoMap = {{9, 10, 11}, {0, 1, 2}, {12, 12, 14}, {15, 16, 17}, {6, 7, 8}, {3, 4, 5}};
void setup()
{
  quadruped.setBodyPosition(0, 0, -40);
  std::cout << "Initial leg positions:" << std::endl;

  Serial.begin(115200);
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

  delay(1500);
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

  double timeStep = 0.05; // Small time increment for smooth stepping
  for (double t = 0; t < stepDuration; t += timeStep)
  {
    double phase = t / stepDuration; // Phase progresses from 0 to 1

    // Move group 1 during the first half of the step
    if (phase < 0.5)
    {
      for (int i : group1)
      {
        auto target = calculateTrajectory(i, phase * 2, stepLength, stepHeight);
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
        auto target = calculateTrajectory(i, (phase - 0.5) * 2, stepLength, stepHeight);
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
  { // Forward swing
    x = defaultX + phase * 2 * stepLength;
  }
  else
  { // Backward placement
    x = defaultX + (1 - (phase - 0.5) * 2) * stepLength;
  }
      //##TODO: FIX everything. It has the traditional shape of the curve but 
      //calculations are wrong. Either fix sinusoidal curves or use bezier curves
      //Fix atachment points and the forward swing, Lift,swing,lower, pull,shift body,continue
      //i presume that im here:     ^^        and i should be here:   ^^
      //                          ^    ^                            ^    ^
      //                         ^^^^^^^^ <-      direction^     ->^^^^^^^^
      //                  direction  <-
  // // Vertical motion (lift/lower)
  double z = defaultZ + (phase <= 0.5 ? stepHeight * sin(M_PI * phase * 2)         // Lift
                                      : stepHeight * sin(M_PI * (1 - phase) * 2)); // Lower

  // // Lateral motion (fixed for crawling)
  // double y = defaultY + 20 * cos(M_PI * phase);
  // double x = defaultX + phase * stepLength;                 // defaultX + stepLength * 0.5 * (1 - cos(2 * M_PI * phase));                                     // Linear horizontal motion
  // double z = defaultZ + stepHeight * sin(2 * M_PI * phase); // Sinusoidal vertical lift
  double y = defaultY + 15 * cos(M_PI * phase);
  return {x, y, z};
}

void Hexapod::setBodyPosition(double x, double y, double z)
{
  bodyX = x; //#TODO: use forward kinematics to get leg pos
  bodyY = y; //then use it to compute adjusted angles based on the step data
  bodyZ = z;//

  // Update the position of each leg to account for the new body position
  for (size_t i = 0; i < legs.size(); ++i)
  {
    // Calculate the default resting position for the leg
    double defaultX = 0;//(i % 2 == 0) ? -L1_TO_R1 / 2 : L1_TO_R1 / 2; // Left legs negative, right legs positive
    double defaultY = 0;//(i < 3) ? L1_TO_L3 / 2 : -L1_TO_L3 / 2;      // Front legs positive, back legs negative
    double defaultZ = LEG_SITTING_Z;
    
    // Adjust the leg's position based on the body's new position
    std::vector<double> targetPosition = {
        defaultX + bodyX, // Adjust forward/backward
        defaultY + bodyY, // Adjust lateral position
        defaultZ + bodyZ  // Adjust height
    };

    // Move the leg to the updated position
    auto angles = legs[i].inverseKinematics(targetPosition);
    // setServoAngles(i, angles);
    // delay(100);
  }
}

void setServoAngles(int legIndex, const std::vector<double> &angles)
{
  // Map joint angles to servo pulses
  auto coxaAng = max(-180.0, min(angles[0] + 8, 180.0));
  auto femurAng = max(-180.0, min(angles[1] - 35, 180.0));
  auto tibiaAng = max(-180.0, min(angles[2] - 83, 180.0));
  if (legIndex % 2 != 0)
  {
    coxaAng = -coxaAng;   //#TODO find better way to mirror angles 
    femurAng = -femurAng; //for the right/left side
    tibiaAng = -tibiaAng; //
  }
  coxa = map(coxaAng, -180, 180, 80, 480);
  femur = map(femurAng, -180, 180, 80, 480);
  tibia = map(tibiaAng, -180, 180, 80, 480);
  // Set servos for the given leg
  pwm.setPWM(legIndex * 3 + 1, 0, femur); // Femur servo
  pwm.setPWM(legIndex * 3 + 2, 0, tibia); // Tibia servo
  pwm.setPWM(legIndex * 3 + 0, 0, coxa);  // Coxa servo
}

void Hexapod::walkCrawl(double stepLength, double stepHeight, double stepDuration)
{
  int legOrder[] = {0, 3, 1, 2}; // Front Left, Back Right, Front Right, Back Left

  for (int legIndex : legOrder)
  {
    // Move one leg at a time
    for (double t = 0; t <= 1.0; t += 0.05)
    {
      double phase = t / 1.0;
      auto target = calculateTrajectory(legIndex, phase, stepLength, stepHeight);
      auto angles = legs[legIndex].inverseKinematics(target);
      setServoAngles(legIndex, angles);
      delay(100);
    }

    // Shift the body slightly to balance the motion
    
    setBodyPosition(bodyX + stepLength / 4, bodyY, bodyZ);
  //#TODO make set body pos work and fix gait generation
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

/*void Hexapod::ForwardMove(float distance, uint16_t steps, uint8_t maxIterations, float epsilon, std::vector<std::vector<int>> ServoMap)
{
  float stepSize = distance / steps;
  /// calculam distanta per pas
  // vom utiliza doua grupuri de picioare
  // vom misca grupul 1 intai
  //   0<|*****|>1        group 1:{0,3,4}
  //   2<|     |>3
  //   4<|*****|>5        group 2:{1,2,5}
  ////////////////////////////////////////////
  // uint8_t GroupA[3]{0, 3, 4};
  // uint8_t GroupB[3]{1, 2, 5};
  std::cout << "Stepsize: " << stepSize;
  uint8_t GroupA[3]{0, 3, 5};
  uint8_t GroupB[3]{1, 2, 4}; /// values for 4 legs
  for (uint16_t step = 0; step < steps; step++)
  {
    // Move Group A
    std::vector<Vector2D> targets(legs.size());

    for (size_t i = 0; i < legs.size(); ++i)
    {
      // Move Group A legs forward by stepSize; others stay stationary
      // targets[i] = (i == 0 || i == 3 || i == 4) ? legs[i].attachmentPoint + Vector2D(stepSize, 0) : legs[i].attachmentPoint;
      targets[i] = (i == 0 || i == 3 || i == 5) ? legs[i].attachmentPoint - Vector2D(0, stepSize) : legs[i].attachmentPoint; // 4legs
      std::cout << "Target " << i << " " << targets[i] << std::endl;
    }
    if (!solveIK(targets, maxIterations, epsilon))
    {
      Serial.println("nope");
      // return;
    } // return; // Abort if IK fails
    // Update servo positions for Group A
    for (uint8_t legIndex : GroupA)
    {
      ControlLeg(legIndex, legs[legIndex].chain, ServoMap);
      delay(500);
    }
    // adjustBodyPos(GroupB);

    // Adjust body position based on Group B
    // Move Group B

    for (size_t i = 0; i < legs.size(); ++i)
    {
      // Move Group B legs forward by stepSize; others stay stationary
      // targets[i] = (i == 1 || i == 2 || i == 5) ? legs[i].attachmentPoint + Vector2D(stepSize, 0) : legs[i].attachmentPoint;
      targets[i] = (i == 1 || i == 2 || i == 4) ? legs[i].attachmentPoint - Vector2D(0, stepSize) : legs[i].attachmentPoint; // 4legs
    }
    if (!solveIK(targets, maxIterations, epsilon))
    {
      Serial.println("nope");
      // return;
    } // return; // Abort if IK fails
    // Update servo positions for Group B
    for (uint8_t legIndex : GroupB)
    {
      ControlLeg(legIndex, legs[legIndex].chain, ServoMap);
      delay(500);
    }

    // Adjust body position based on Group A
    // adjustBodyPos(GroupA);
  }
}*/
