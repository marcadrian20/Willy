#include "WillyControllersAndInterfaces.h"
#include "WillyIK.h"

BalanceController::BalanceController(Hexapod& hexapod)
    : pitchPID(2.0, 0.1, 1.0, 0.0),
      rollPID(2.0, 0.1, 1.0, 0.0),
      robot(hexapod)
{
}
void BalanceController::balance(float pitch, float roll)
{
    float xCorrection = pitchPID.calculate(pitch);
    float yCorrection = rollPID.calculate(roll);
    
    // Constrain corrections
    xCorrection = constrain(xCorrection, -20, 20);
    yCorrection = constrain(yCorrection, -20, 20);

    // Apply corrections through body position
    robot.setBodyPosition(xCorrection, yCorrection, LEG_SITTING_Z);
}
void BalanceController::reset()
{
    pitchPID.reset();
    rollPID.reset();
}