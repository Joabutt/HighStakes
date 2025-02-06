#include "auton.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include "helper.hpp"
#include "robot.hpp"

void red_negative(void)
{
    move(0.01, -10, -10, false);  // move to stake
    vexDelay(100);                // wait for robot to reach stake
    mogo(true);                   // clamp stake
    vexDelay(500);                // wait for stake to be clamped
    roller_spin(100);             // intake starting donut
    move(195, 20, -20, false);    // turn to red understacked donut
    move(220, 20, 20, false);     // move to red understacked donut & intake
    vexDelay(800);                // wait for donut to be intaken
    move(110, 20, -20, false);    // turn to red understacked donut at white line
    move(200, 20, 20, false);     // move to red understacked donut at white line & intake
    move(40, -20, -20, false);    // move back
    vexDelay(800);                // wait for donut to be intaken
    move(7, -20, 20, false);      // turn to second red understacked donut at white line
    move(200, 20, 20, false);     // move to second red understacked donut at white line & intake
    vexDelay(800);                // wait for donut to be intaken
    move(950, -27.5, -15, false); // turn to bar and touch
    roller_spin(0);               // stop rollers
}

// Autonomous route for Red Negative
void red_positive(void)
{
    /*
    move(630, -30, -30, false);  // move to stake
    vexDelay(100);               // wait for robot to reach stake
    mogo(true);                  // clamp stake
    vexDelay(500);               // wait for stake to be clamped
    roller_spin(100);            // wait for stake to be clamped
    move(200, -20, 20, false);   // turn to red understacked donut
    move(290, 20, 20, false);    // move to red understacked donut & intake
    vexDelay(1800);              // wait for donut to be intaken
    move(50, -20, -20, false);   // move back
    move(70, 25, -25, false);    // turn towards positive corner
    move(200, 30, 30, false);    // move to positive corner
    move(390, 60, 29, false);    // arc towards positive corner
    roller_spin(0);              // stop rollers
    move(120, 30, 30, false);    // move to positive corner
    move(500, -20, 20, false);   // turn
    mogo(false);                 // unclamp stake
    move(640, 10, 50, false);    // turn out
    move(1000, -40, -40, false); // move towards stake
    mogo(true);                  // clamp stake
    vexDelay(500);               // wait for stake to be clamped
    move(1300, 60, 30, false);    // move towards bar

    dink(true); // open dinkler
    move(600, 30, 30, false);    // move to positive corner
    move(300, -30, 30, false); // clear positive corner
    dink(false); */
}

// Autonomous route for Blue Positive
void blue_negative(void)
{
    /*
    move(630, -30, -30, false);  // move to stake
    vexDelay(100);               // wait for robot to reach stake
    mogo(true);                  // clamp stake
    vexDelay(500);               // wait for stake to be clamped
    roller_spin(100);            // wait for stake to be clamped
    */
}

// Autonomous route for Blue Negative
void blue_positive(void)
{
    /*
    move(6, -30, -30, false);  // move to stake
    vexDelay(100);               // wait for robot to reach stake
    mogo(true);                  // clamp stake
    vexDelay(500);               // wait for stake to be clamped
    roller_spin(100);            // wait for stake to be clamped
    */
}

void auton(void)
{
    red_negative();
}

void skill(void)
{
    // Implement your skills logic here
}

const double kP = 0.8;
const double kI = 0.0;
const double kD = 0.15;
const double MIN_DEGREE_THRESHOLD = 1.0; // Minimum degree threshold
const double TIMEOUT = 5000;             // Timeout in milliseconds
const double DEGREE_MULTIPLIER = 0.01;   // Multiplier to scale down the degree value

// Custom clamp function
template <typename T>
T clamp(T value, T min, T max)
{
    if (value < min)
        return min;
    if (value > max)
        return max;
    return value;
}

void move(double degree, double left, double right, bool invert) {
    if (invert) std::swap(left, right);
    if (cfg.rev) goto negate;
    if (degree < 0) {
        degree = -degree;

    negate:
        left = -left;
        right = -right;
    }

    // Reset motor encoders
    for (uint8_t i = 0; i < MOTORS_BASE; i++) {
        robo_g.base[i].resetPosition();
    }

    double error = 0, lastError = 0, integral = 0, derivative = 0;
    double output = 0;
    vex::timer moveTimer;
    moveTimer.clear();

    while (std::fabs(robo_g.base[LF].position(vex::rotationUnits::deg)) < degree) {
        double currentPosition = std::fabs(robo_g.base[LF].position(vex::rotationUnits::deg));
        error = degree - currentPosition;
        integral = (fabs(error) < 5) ? integral + error : 0;
        derivative = error - lastError;
        output = (kP * error) + (kI * integral) + (kD * derivative);
        output = clamp(output, -100.0, 100.0);
        lastError = error;

        // **Gyro Correction**
        double currentHeading = robo_g.gyro.rotation(vex::rotationUnits::deg);
        double headingError = 0 - currentHeading; // Keeping it straight
        double headingCorrection = headingError * 0.5; 

        // Apply PID + gyro correction to motor speeds
        for (uint8_t i = 0; i < MOTORS_BASE; i++) {
            double correctedSpeed = (i < drive_motors::RF) ? (left + output - headingCorrection)
                                                           : (right + output + headingCorrection);
            robo_g.base[i].spin(vex::directionType::fwd, correctedSpeed, vex::percentUnits::pct);
        }

        // Timeout check
        if (moveTimer.time(vex::timeUnits::msec) > TIMEOUT) break;
        vex::task::sleep(25);
    }

    // Stop motors
    for (uint8_t i = 0; i < MOTORS_BASE; i++) {
        robo_g.base[i].stop(vex::brakeType::brake);
    }
}


void turn_until(double degree, double leftSpeed, double rightSpeed, bool invert, double calibrationFactor)
{
    // Reset gyroscope and odometry readings
    robo_g.gyro.resetRotation();

    double targetAngle = fabs(degree);
    double travelledAngle = 0;

    if (invert)
        std::swap(leftSpeed, rightSpeed);
    if (cfg.rev)
    {
        leftSpeed = -leftSpeed;
        rightSpeed = -rightSpeed;
        targetAngle = -targetAngle;
    }
    if (leftSpeed < rightSpeed)
        targetAngle = -targetAngle;

    for (uint8_t i = 0; i < MOTORS_BASE; i++)
    {
        robo_g.base[i].spin(vex::directionType::fwd,
                            i < drive_motors::RF ? leftSpeed : rightSpeed,
                            vex::percentUnits::pct);
    }

    while (fabs(travelledAngle) < fabs(targetAngle))
    {
        // updateOdometry();
        travelledAngle = robo_g.gyro.rotation(vex::rotationUnits::deg) * calibrationFactor;

        // Adjust motor speed if needed (optional)
        if (fabs(travelledAngle) > fabs(targetAngle) * 0.9)
        { // Slow down as approaching target
            for (uint8_t i = 0; i < MOTORS_BASE; i++)
            {
                robo_g.base[i].spin(vex::directionType::fwd,
                                    i < drive_motors::RF ? leftSpeed * 0.5 : rightSpeed * 0.5,
                                    vex::percentUnits::pct);
            }
        }

        vex::task::sleep(25);
    }

    for (uint8_t i = 0; i < MOTORS_BASE; i++)
    {
        robo_g.base[i].stop(vex::brakeType::brake);
    }
}