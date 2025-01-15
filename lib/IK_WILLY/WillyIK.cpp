#include "WillyIK.h"

std::vector<double> Hexapod::calculateTrajectory(int legIndex, double phase, double stepLength, double stepHeight, int motionType, int direction)
{ // Neutral positions for the leg
    double defaultX = L1_TO_R1 / 2;
    double defaultY = L1_TO_L3 / 2;
    double defaultZ = LEG_SITTING_Z + 10;
    // Direction 0 is forward, 1 is backward, 2 is left, 3 is right
    //  X is Horizontal motion (forward/backward)
    //  Y is Lateral motion (side to side)
    //  Z is Vertical motion (up and down)
    //  Motion type 0 is forward/backward, 1 is side to side,if motion type is 2, 0 is left, 1 is right
    double x = defaultX, y = defaultY;
    // x = 0;
    std::cout << "Leg index" << legIndex << "Phase" << phase << std::endl; // 0.625
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

void Hexapod::setLegServoAngles(int legIndex, const std::vector<double> &angles)
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
    servoController.setAngle(legIndex * 3 + 0, coxaAng);
    servoController.setAngle(legIndex * 3 + 1, femurAng);
    servoController.setAngle(legIndex * 3 + 2, tibiaAng);
    std::cout << "^^^^^^^^^^^^^^^^" << std::endl;
    // Set servos for the given leg
    //   // Set servos for the given leg
    //   pwm.setPWM(legIndex * 3 + 1, 0, femur); // Femur servo
    //   // std::cout << coxa << " " << femur << " " << tibia << std::endl;
    //   pwm.setPWM(legIndex * 3 + 2, 0, tibia); // Tibia servo
    //   // std::cout << coxa << " " << femur << " " << tibia << std::endl;
    //   pwm.setPWM(legIndex * 3 + 0, 0, coxa); // Coxa servo
    //   // std::cout << coxa << " " << femur << " " << tibia << std::endl;
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
        double defaultX = L1_TO_R1 / 2; // L1_TO_R1 / 2;
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
            setLegServoAngles(i, angles);
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
            setLegServoAngles(legIndex, angles);
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
    static int balanceCounter = 0;
    // Single gait cycle
    for (double t = 0; t <= 1.0; t += 0.1)
    {
        // Move all legs according to their phases
        for (int i = 0; i < 4; i++)
        {
            double legPhase = t + legPhases[i];
            while (legPhase >= 1.0)
                legPhase -= 1.0;

            auto target = calculateTrajectory(i, legPhase, stepLength, stepHeight, motionType, direction);
            // currentLegTargets[i] = target; // Store the current target for each leg
            auto angles = legs[i].inverseKinematics(target);
            setLegServoAngles(i, angles);
            // balanceBody();
        }
        // if (balanceCounter % 5 == 0)
        // {
        //     // delay(50);
        //     balanceBody();
        // }
        // balanceCounter++;
        // balanceBody();
        delay(stepDuration * 50); // Adjust timing as needed
    }
}

void Hexapod::initializeStance()
{
    // Define neutral positions for each leg
    std::vector<std::vector<double>> neutralPositions = {
        {L1_TO_R1 / 2, L1_TO_L3 / 2, LEG_SITTING_Z + 10}, // Front Left (L1)
        {L1_TO_R1 / 2, L1_TO_L3 / 2, LEG_SITTING_Z + 10}, // Front Right (R1)
        {L1_TO_R1 / 2, L1_TO_L3 / 2, LEG_SITTING_Z + 10}, // Rear Left (L3)
        {L1_TO_R1 / 2, L1_TO_L3 / 2, LEG_SITTING_Z + 10}, // Rear Right (R3)
        {-L1_TO_R1 / 2, 0, LEG_SITTING_Z},                // Middle Left (L2)
        {L1_TO_R1 / 2, 0, LEG_SITTING_Z}                  // Middle Right (R2)
    };

    // Apply IK to move each leg to its neutral position
    for (size_t i = 0; i < legs.size(); ++i)
    {
        auto angles = legs[i].inverseKinematics(neutralPositions[i]);
        setLegServoAngles(i, angles); // Send angles to the servos
    }
}

void Hexapod::InitializeRobotControllers()
{
    servoController.init();
    mpu6500.init();
}

void Hexapod::sittingAction()
{
    // Define neutral positions for each leg
    std::vector<std::vector<double>> neutralPositions = {
        {L1_TO_R1 / 2, L1_TO_L3 / 2, -60}, // Front Left (L1)
        {L1_TO_R1 / 2, L1_TO_L3 / 2, -60}, // Front Right (R1)
        {L1_TO_R1 / 2, L1_TO_L3 / 2, -60}, // Rear Left (L3)
        {L1_TO_R1 / 2, L1_TO_L3 / 2, -60}, // Rear Right (R3)
        {-L1_TO_R1 / 2, 0, LEG_SITTING_Z},                // Middle Left (L2)
        {L1_TO_R1 / 2, 0, LEG_SITTING_Z}                  // Middle Right (R2)
    };

    // Apply IK to move each leg to its neutral position
    for (size_t i = 0; i < legs.size(); ++i)
    {
        auto angles = legs[i].inverseKinematics(neutralPositions[i]);
        setLegServoAngles(i, angles); // Send angles to the servos
    }
    // delay(1000);
    // setBodyPosition(0, 0, 0);
    // delay(1000);
    // setBodyPosition(0, 0, 5);
    // delay(1000);
    // setBodyPosition(0, 0, 0);
    // delay(1000);
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
            setLegServoAngles(i, angles);
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
            setLegServoAngles(i, angles);
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

////////////Spider leg class
std::vector<std::vector<double>> SpiderLeg::forwardKinematics()
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
std::vector<double> SpiderLeg::inverseKinematics(const std::vector<double> &target)
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

void Hexapod::balanceBody()
{
    float currentPitch = mpu6500.getPitch();
    float currentRoll = mpu6500.getRoll();
    std::cout << "Current pitch: " << currentPitch << std::endl;
    std::cout << "Current roll: " << currentRoll << std::endl;
    std::cout << "Balancing body..." << std::endl;
    // std::cout<<"before modify:"<<currentLegTargets[i][2]<<std::endl;
    balanceController.balance(currentPitch, currentRoll);
}