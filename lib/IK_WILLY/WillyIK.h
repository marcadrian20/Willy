#pragma once
#include "Definitions.h"
#include "WillyControllersAndInterfaces.h"

class SpiderLeg
{
private:
    std::string name;
    double COXA, FEMUR, TIBIA;
    double theta1, theta2, theta3;
    std::vector<std::vector<double>> joints;

public:
    SpiderLeg(std::string legName, double coxa, double femur, double tibia)
        : name(legName), COXA(coxa), FEMUR(femur), TIBIA(tibia),
          theta1(0), theta2(0), theta3(0) {}

    // Normalize angles to range [-180, 180]
    double normalizeAngle(double angle)
    {
        while (angle > 180)
            angle -= 360;
        while (angle < -180)
            angle += 360;
        return angle;
    }

    // Set joint angles
    void setAngles(const std::vector<double> &angles)
    {
        if (angles.size() != 3)
            throw std::invalid_argument("Angles vector must have 3 elements.");
        theta1 = normalizeAngle(angles[0]);
        theta2 = normalizeAngle(angles[1]);
        theta3 = normalizeAngle(angles[2]);
    }

    // Get current joint angles
    std::vector<double> getAngles() const
    {
        return {theta1, theta2, theta3};
    }

    // Forward kinematics to compute joint positions
    std::vector<std::vector<double>> forwardKinematics();
    std::vector<double> inverseKinematics(const std::vector<double> &target);

    // Print joint positions
    void printJointPositions()
    {
        auto positions = forwardKinematics();
        std::cout << "Joint positions:" << std::endl;
        for (const auto &pos : positions)
        {
            std::cout << "(" << pos[0] << ", " << pos[1] << ", " << pos[2] << ")" << std::endl;
        }
    }
};

class Hexapod
{
private:
    std::vector<SpiderLeg> legs;
    double bodyX, bodyY, bodyZ; // Body's current position
    double orientation;         // Body's orientation in degrees
    ServoController servoController=ServoController(PCA9685_ADDRESS);
    MPU6500Interface mpu6500=MPU6500Interface(MPU6500_ADDRESS);
    BalanceController balanceController=BalanceController(*this);
    
public:
    // Constructor to initialize legs and body position
    std::vector<std::vector<double>> currentLegTargets;
    Hexapod(double coxa, double femur, double tibia) : bodyX(0), bodyY(0), bodyZ(LEG_SITTING_Z), orientation(0)
    {
        // Initialize 6 legs with default names and dimensions
        for (int i = 0; i < 6; i++) // 6
        {
            legs.emplace_back("Leg" + std::to_string(i + 1), coxa, femur, tibia);
        }
         currentLegTargets.resize(6, {L1_TO_R1/2, L1_TO_L3/2, LEG_SITTING_Z+10});
    }
    // Get references to all legs
    std::vector<SpiderLeg> &getLegs()
    {
        return legs;
    }
    // Set the body position
    void setBodyPosition(double x, double y, double z);
    void initializeStance();// Initialize the robot's stance ,also the standing position/action
    // Perform a walking gait
    void InitializeRobotControllers();
    // Print the positions of all legs
    void printLegPositions()
    {
        for (int i = 0; i < legs.size(); i++)
        {
            std::cout << "Leg " << i + 1 << ":" << std::endl;
            legs[i].printJointPositions();
        }
    }
    float getRoll()
    {
        return mpu6500.getRoll();
    }
    float getPitch()
    {
        return mpu6500.getPitch();
    }
    float getTemp()
    {
        return mpu6500.getTemp();
    }
    void SetPidTargets(float pitch, float roll)
    {
        balanceController.setTarget(pitch, roll);
    }
    void balanceBody();
    // Set the angles of the servos for a given leg
    void setLegServoAngles(int legIndex, const std::vector<double> &angles);
    void walkQuadruped(double stepLength, double stepHeight, double stepDuration, int motionType, int direction);
    std::vector<double> calculateTrajectory(int legIndex, double phase, double stepLength, double stepHeight, int motionType, int direction);
    std::vector<double> calculateWaveTrajectory(int legIndex, double phase, double stepLength, double stepHeight, int motionType, int direction);
    void walkCrawl(double stepLength, double stepHeight, double stepDuration, int motionType, int direction);
    void walkWaveGait(double stepLength, double stepHeight, double stepDuration, int motionType, int direction);
    void sittingAction();
};
