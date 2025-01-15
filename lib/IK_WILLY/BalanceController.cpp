#include "WillyControllersAndInterfaces.h"
#include "WillyIK.h"

BalanceController::BalanceController(Hexapod &hexapod)
    : pitchPID(1.5, 0.0, 0.0, 0.0),
      rollPID(1.5, 0.00, 0.0, 0.0),
      robot(hexapod)
{
}
void BalanceController::balance(float pitch, float roll)
{
    float PitchCorrection = pitchPID.calculate(pitch);
    float RollCorrection = rollPID.calculate(roll);

    // Constrain corrections
    PitchCorrection = constrain(PitchCorrection, -30, 30);
    RollCorrection = constrain(RollCorrection, -30, 30);
    // std::cout << "ROLL CORRECTION: " << RollCorrection << std::endl;
    // double baseHeight = LEG_SITTING_Z + 10;
    for (int i = 0; i < 4; i++)
    {
        // double z = baseHeight;
        double x = robot.currentLegTargets[i][0]; // Current x-coordinate
        double y = robot.currentLegTargets[i][1]; // Current y-coordinate
        // std::cout << "current debug target:" << robot.currentLegTargets[i][2] << std::endl;
        double z = robot.currentLegTargets[i][2]; // Current z-coordinate
        // Front legs (0, 1)
        if (i == 0 || i == 1)
        {
            z -= PitchCorrection; // Positive pitch -> lower front
        }
        // Back legs (2, 3)
        else
        {
            z += PitchCorrection; // Positive pitch -> raise back
        }

        if (i == 1 || i == 3)
        {
            z -= RollCorrection; // Positive roll -> raise right legs
        }
        // Left legs (0, 2)
        else
        {
            z += RollCorrection; // Positive roll -> lower left legs
        }
        // Apply final height with safety constraints
        z = constrain(z, LEG_SITTING_Z - 30, LEG_SITTING_Z + 30);
        // std::cout << "Leg " << i << " : " << " " << z << std::endl;
        auto angles = robot.getLegs()[i].inverseKinematics({x, y, z});
        robot.currentLegTargets[i][2] = z;
        // std::cout << "current target:" << robot.currentLegTargets[i][2] << std::endl;
        // angles[2]+=PitchCorrection;
        // angles[1]+=RollCorrection;
        robot.setLegServoAngles(i, angles);
    }
    // auto correctedAngles=robot.getLegs()[i].inverseKinematics({currentPos[4][0],currentPos[4][1],z});
    // robot.setLegServoAngles(i,correctedAngles);
    // std::cout<<"Leg "<<i<<"corrected target: "<<currentPos[4][0]<<" "<<currentPos[4][1]<<"w "<<z<<std::endl;
    // if pitch is positive,the bot is tilted forward
    // if pitch is negative, the bot is tilted backward
    // if roll is positive, the bot is tilted to the right
    // if roll is negative, the bot is tilted to the left
    //  Apply corrections through body position
    //  robot.setBodyPosition(xCorrection, yCorrection, LEG_SITTING_Z);
    // find way to adjust the height of the robot based on pitch and roll

    // std::cout << "Pitch Correction: " << PitchCorrection << std::endl;
    // std::cout << "Roll Correction: " << RollCorrection << std::endl;
    // std::cout << "Pitch: " << pitch << std::endl;
    // std::cout << "Roll: " << roll << std::endl;
}
void BalanceController::reset()
{
    pitchPID.reset();
    rollPID.reset();
}
void BalanceController::setTarget(float pitch, float roll)
{
    pitchPID.setTarget(pitch);
    rollPID.setTarget(roll);
}