#include "WillyIK.h"


Hexapod quadruped(43, 60, 104);


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
  while(!Serial);
  quadruped.initializeStance();
  delay(2000);
  quadruped.InitializeRobotControllers();
  delay(1000);
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
  if (Serial.available()) // if reads 'w' then walk, 'r' rotate
  {
    char command = Serial.read();
    if (command == 'w')
    {
      quadruped.walkWaveGait(stepLength, stepHeight, stepDuration, WALK, WALK_FORWARD);
    }
    else if (command == 's')
    {
      quadruped.walkWaveGait(stepLength, stepHeight, stepDuration, WALK, WALK_BACKWARDS);
    }
    else if (command == 'a')
    {
      quadruped.walkWaveGait(stepLength, stepHeight, stepDuration, ROTATE, ROTATE_LEFT);
    }
    else if (command == 'd')
    {
      quadruped.walkWaveGait(stepLength, stepHeight, stepDuration, ROTATE, ROTATE_RIGHT);
    }
    // else if (command == 'q')
    // {
    //   quadruped.walkQuadruped(stepLength, stepHeight, stepDuration, WALK, 0);
    // }
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



void adjustPosture()
{
  // Adjust the posture of the robot to ensure stability
  // This function can be used to correct the robot's posture
  // after a step or during a gait cycle.
  //get current pitch and roll
  // float pitch = mpu6500.getPitch();
  // float roll = mpu6500.getRoll();
  // Serial.print("Pitch   = ");
  // Serial.print(pitch);
  // Serial.print("  |  Roll    = ");
  // Serial.println(roll);
  // Serial.println();

  //get current position using forward kinematics
  //then adjust the position to correct the posture
  //shift the body to the opposite direction of the tilt
  //if pitch is positive, the robot is tilted forward
  //if pitch is negative, the robot is tilted backward
  //if roll is positive, the robot is tilted to the right
  //if roll is negative, the robot is tilted to the left
  //adjust the body position to correct the tilt
  //if pitch is positive, move the body backward
  //if pitch is negative, move the body forward
  //if roll is positive, move the body to the left
  //if roll is negative, move the body to the right
  //#TODO convert delay to milis
  
  // delay(1000);
}

