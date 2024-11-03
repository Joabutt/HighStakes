#include "auton.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include "helper.hpp"
#include "robot.hpp"


Position robotPosition = {0, 0, 0};  // Initialize position

/* void updateOdometry() {
    static double lastLeft = 0, lastRight = 0, lastHorizontal = 0;
    double currentLeft = robo_g.leftRtn.position(vex::rotationUnits::deg);
    double currentHorizontal = robo_g.horizRtn.position(vex::rotationUnits::deg);

    double deltaLeft = (currentLeft - lastLeft) * M_PI / 180.0;
    double deltaHorizontal = (currentHorizontal - lastHorizontal) * M_PI / 180.0;

    lastLeft = currentLeft;
    lastHorizontal = currentHorizontal;

    double deltaTheta = deltaHorizontal / HORIZONTAL_WHEEL_OFFSET;
    double deltaY = deltaLeft;
    double deltaX = deltaHorizontal - deltaTheta * HORIZONTAL_WHEEL_OFFSET;

    robotPosition.theta += deltaTheta;
    robotPosition.x += deltaX * cos(robotPosition.theta) - deltaY * sin(robotPosition.theta);
    robotPosition.y += deltaX * sin(robotPosition.theta) + deltaY * cos(robotPosition.theta);

    // Bound coordinates to the field dimensions
    robotPosition.x = std::max(0.0, std::min(robotPosition.x, 144.0));
    robotPosition.y = std::max(0.0, std::min(robotPosition.y, 144.0));

    // Log the coordinates
    std::cout << "X: " << robotPosition.x << " Y: " << robotPosition.y << " Theta: " << robotPosition.theta * 180.0 / M_PI << " degrees" << std::endl;
}
*/ 

void auton(void) {
    move(620, -30, -30, false);
    wing(true);
    vexDelay(500);
    roller_spin(100);
    vexDelay(500);
    move(90, 10, -10, false);
    roller_spin(0);
    move(500, -30, -30, true);
    move(-100, -30, -30, false);
}

void skill(void) {
    // Implement your skills logic here
}

void move(double degree, double left, double right, bool invert) {
    double la, ra; /**< fabsolute velocites for left and right */
    uint8_t fi;    /**< index of faster side */

    if (invert) std::swap(left, right);
    if (cfg.rev) goto negate;
    if (degree < 0) {
        degree = -degree;

    negate:
        left = -left;
        right = -right;
    }

    la = std::fabs(left);
    ra = std::fabs(right);
    fi = la > ra ? LF : RF;

    robo_g.base[fi].resetPosition();

    for (uint8_t i = 0; i < MOTORS_BASE; i++)
        robo_g.base[i].spin(vex::directionType::fwd,
                            i < drive_motors::RF ? left : right,
                            vex::percentUnits::pct);

    while (std::fabs(robo_g.base[fi].position(vex::rotationUnits::deg)) <
           degree)
        vex::task::sleep(25);

    for (uint8_t i = 0; i < MOTORS_BASE; i++)
        robo_g.base[i].stop(vex::brakeType::brake);
}

void turn_until(double degree, double leftSpeed, double rightSpeed, bool invert, double calibrationFactor) {
    // Reset gyroscope and odometry readings
    robo_g.gyro.resetRotation();
    robotPosition.theta = 0;

    double targetAngle = fabs(degree);
    double travelledAngle = 0;

    if (invert) std::swap(leftSpeed, rightSpeed);
    if (cfg.rev) {
        leftSpeed = -leftSpeed;
        rightSpeed = -rightSpeed;
        targetAngle = -targetAngle;
    }
    if (leftSpeed < rightSpeed) targetAngle = -targetAngle;

    for (uint8_t i = 0; i < MOTORS_BASE; i++) {
        robo_g.base[i].spin(vex::directionType::fwd,
                            i < drive_motors::RF ? leftSpeed : rightSpeed,
                            vex::percentUnits::pct);
    }

    while (fabs(travelledAngle) < fabs(targetAngle)) {
        //updateOdometry();
        travelledAngle = robo_g.gyro.rotation(vex::rotationUnits::deg) * calibrationFactor;

        // Adjust motor speed if needed (optional)
        if (fabs(travelledAngle) > fabs(targetAngle) * 0.9) {  // Slow down as approaching target
            for (uint8_t i = 0; i < MOTORS_BASE; i++) {
                robo_g.base[i].spin(vex::directionType::fwd,
                                    i < drive_motors::RF ? leftSpeed * 0.5 : rightSpeed * 0.5,
                                    vex::percentUnits::pct);
            }
        }

        vex::task::sleep(25);
    }

    for (uint8_t i = 0; i < MOTORS_BASE; i++) {
        robo_g.base[i].stop(vex::brakeType::brake);
    }
}
