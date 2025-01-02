#define LEG_SITTING_Z -40.0
#pragma once
#define _USE_MATH_DEFINES
#include <iostream>
#include <cmath>
#include <vector>
#include <stdexcept>
#include <string>

const double L1_TO_R1 = 126;
const double L1_TO_L3 = 167;
const double L2_TO_R2 = 163;

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
    std::vector<std::vector<double>> forwardKinematics()
    {
        double radTheta1 = theta1 * M_PI / 180.0;
        double radTheta2 = theta2 * M_PI / 180.0;
        double radTheta3 = theta3 * M_PI / 180.0;

        double Xa = COXA * cos(radTheta1);
        double Ya = COXA * sin(radTheta1);
        double G2 = sin(radTheta2) * FEMUR;
        double P1 = cos(radTheta2) * FEMUR;

        double Xc = cos(radTheta1) * P1;
        double Yc = sin(radTheta1) * P1;
        double H = sqrt(TIBIA * TIBIA + FEMUR * FEMUR -
                        2 * TIBIA * FEMUR * cos(M_PI - radTheta3));
        double phi1 = acos(std::max(-1.0, std::min(1.0, (FEMUR * FEMUR + H * H - TIBIA * TIBIA) / (2 * FEMUR * H))));
        double phi3 = phi1 - radTheta2;
        double xb = cos(radTheta1) * cos(phi3) * H;
        double yb = sin(radTheta1) * cos(phi3) * H;
        double G1 = -sin(phi3) * H;

        joints = {
            {0, 0, 0},
            {Xa, Ya, 0},
            {Xa + cos(radTheta1) * P1, Ya + sin(radTheta1) * P1, G2},
            {Xa + xb, Ya + yb, G1}};

        return joints;
    }

    // Inverse kinematics to calculate joint angles for a target position
    std::vector<double> inverseKinematics(const std::vector<double> &target)
    {
        if (target.size() != 3)
            throw std::invalid_argument("Target must have 3 elements.");

        double x = target[0], y = target[1], z = target[2];

        double theta1 = atan2(y, x);    // * 180.0 / M_PI;
        double Xa = COXA * cos(theta1); // * M_PI / 180.0);
        double Ya = COXA * sin(theta1); // * M_PI / 180.0);

        double Xb = x - Xa;
        double Yb = y - Ya;
        double P = Xb / cos(theta1); //* M_PI / 180.0);

        double G = std::fabs(z);
        double H = sqrt(P * P + G * G);

        if (H > (FEMUR + TIBIA))
        {
            std::cerr << "Target out of reach!" << std::endl;
            H = FEMUR + TIBIA;
        }

        double phi3 = asin(G / H);
        double phi2 = acos(std::max(-1.0, std::min(1.0, (TIBIA * TIBIA + H * H - FEMUR * FEMUR) /
                                                            (2 * TIBIA * H))));

        double phi1 = acos(std::max(-1.0, std::min(1.0, (pow(FEMUR, 2) + pow(H, 2) - pow(TIBIA, 2)) /
                                                            (2 * FEMUR * H))));

        double theta2 = (z > 0) ? phi1 + phi3 : phi1 - phi3;
        double theta3 = phi1 + phi2;

        setAngles({theta1 * 180.0 / M_PI, theta2 * 180.0 / M_PI, theta3 * 180.0 / M_PI});
        return getAngles();
    }

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

public:
    // Constructor to initialize legs and body position
    Hexapod(double coxa, double femur, double tibia) : bodyX(0), bodyY(0), bodyZ(LEG_SITTING_Z), orientation(0)
    {
        // Initialize 6 legs with default names and dimensions
        for (int i = 0; i < 6; i++) // 6
        {
            legs.emplace_back("Leg" + std::to_string(i + 1), coxa, femur, tibia);
        }
    }

    // Get references to all legs
    std::vector<SpiderLeg> &getLegs()
    {
        return legs;
    }

    // Set the body position
    void setBodyPosition(double x, double y, double z);
    // {
    //     bodyX = x;
    //     bodyY = y;
    //     bodyZ = z;
    // }

    // Move a leg to a target position
    void moveLeg(int legIndex, const std::vector<double> &target)
    {
        if (legIndex < 0 || legIndex >= legs.size())
        {
            std::cerr << "Invalid leg index: " << legIndex << std::endl;
            return;
        }

        auto angles = legs[legIndex].inverseKinematics(target);
        if (!angles.empty())
        {
            std::cout << "Leg " << legIndex + 1 << " moved to target: ";
            for (const auto &angle : angles)
            {
                std::cout << angle << " ";
            }
            std::cout << std::endl;
        }
    }
    void initializeStance();
    // Perform a walking gait
    void walk(double stepLength, double stepHeight)
    {
        // Example: tripod gait (three legs move at a time)
        int group1[] = {0, 3, 5}; // Legs 1, 4, 6
        int group2[] = {1, 2, 4}; // Legs 2, 3, 5

        // Step 1: Move group 1 forward
        for (int i : group1)
        {
            moveLeg(i, {bodyX + stepLength, bodyY, bodyZ + stepHeight});
        }
        // Simulate body shift (all legs support the weight here)
        setBodyPosition(bodyX + stepLength / 2, bodyY, bodyZ);

        // Step 2: Move group 2 forward
        for (int i : group2)
        {
            moveLeg(i, {bodyX + stepLength, bodyY, bodyZ + stepHeight});
        }
        // Finalize step
        setBodyPosition(bodyX + stepLength, bodyY, bodyZ);
    }

    // Print the positions of all legs
    void printLegPositions()
    {
        for (int i = 0; i < legs.size(); i++)
        {
            std::cout << "Leg " << i + 1 << ":" << std::endl;
            legs[i].printJointPositions();
        }
    }
    void walkQuadruped(double stepLength, double stepHeight, double stepDuration);
    std::vector<double> calculateTrajectory(int legIndex, double phase, double stepLength, double stepHeight);
    void walkCrawl(double stepLength, double stepHeight, double stepDuration); 
};
