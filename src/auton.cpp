#include "auton.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include "helper.hpp"
#include "robot.hpp"

PIDController pid(0.5, 0.0, 0.1);  // Adjust these values through testing

void auton(void) {
    move(620, -30, -30, true);
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

double clamp(double value, double min, double max) {
    return std::min(std::max(value, min), max);
}

void move(double degree, double left, double right, bool invert) {
    double la, ra;
    uint8_t fi;

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
    double targetPosition = degree;
    double currentPosition = 0;
    double deltaTime = 0.025;  // 25ms control loop

    while (std::fabs(currentPosition) < targetPosition) {
        currentPosition = std::fabs(robo_g.base[fi].position(vex::rotationUnits::deg));

        // Compute the motor speed using PID
        double motorSpeed = pid.calculate(targetPosition, currentPosition, deltaTime);

        // Set motor speeds with calculated PID output
        for (uint8_t i = 0; i < MOTORS_BASE; i++) {
            robo_g.base[i].spin(vex::directionType::fwd,
                                i < drive_motors::RF ? motorSpeed : motorSpeed,
                                vex::percentUnits::pct);
        }

        // Delay for the loop
        vex::task::sleep(25);
    }

    // Stop all motors
    for (uint8_t i = 0; i < MOTORS_BASE; i++)
        robo_g.base[i].stop(vex::brakeType::brake);
}


void turn_until(double degree, double leftSpeed, double rightSpeed, bool invert, double calibrationFactor) {
    // Reset gyroscope and odometry readings
    robo_g.gyro.resetRotation();

    // Create a PID controller specifically for turning
    PIDController pidTurn(1.0, 0.0, 0.1);  // Tuning values for turn control
    double targetAngle = fabs(degree);
    double travelledAngle = 0;
    double deltaTime = 0.025;  // 25ms control loop
    double maxSpeed = std::max(fabs(leftSpeed), fabs(rightSpeed));

    if (invert) std::swap(leftSpeed, rightSpeed);
    if (cfg.rev) {
        leftSpeed = -leftSpeed;
        rightSpeed = -rightSpeed;
        targetAngle = -targetAngle;
    }
    if (leftSpeed < rightSpeed) targetAngle = -targetAngle;

    while (fabs(travelledAngle) < fabs(targetAngle)) {
        travelledAngle = robo_g.gyro.rotation(vex::rotationUnits::deg) * calibrationFactor;

        // Calculate motor speed based on PID controller
        double pidOutput = pidTurn.calculate(targetAngle, travelledAngle, deltaTime);
        pidOutput = clamp(pidOutput, -maxSpeed, maxSpeed);

        // Set motor speeds for turning with PID adjustments
        for (uint8_t i = 0; i < MOTORS_BASE; i++) {
            double adjustedSpeed = (i < drive_motors::RF ? pidOutput : -pidOutput);
            robo_g.base[i].spin(vex::directionType::fwd, adjustedSpeed, vex::percentUnits::pct);
        }

        // Delay for the loop
        vex::task::sleep(25);
    }

    // Stop all motors once the target angle is reached
    for (uint8_t i = 0; i < MOTORS_BASE; i++) {
        robo_g.base[i].stop(vex::brakeType::brake);
    }
}
