#include "auton.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include "helper.hpp"
#include "robot.hpp"

void red_negative(void)
{
    move(630, -30, -30, false);   // move to stake
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

    /* dink(true); // open dinkler
    move(600, 30, 30, false);    // move to positive corner
    move(300, -30, 30, false); // clear positive corner
    dink(false); */
}

// Autonomous route for Blue Positive
void blue_negative(void)
{
    move(630, -30, -30, false);  // move to stake
    vexDelay(100);               // wait for robot to reach stake
    mogo(true);                  // clamp stake
    vexDelay(500);               // wait for stake to be clamped
    roller_spin(100);            // wait for stake to be clamped
}

// Autonomous route for Blue Negative
void blue_positive(void)
{
    move(6, -30, -30, false);  // move to stake
    vexDelay(100);               // wait for robot to reach stake
    mogo(true);                  // clamp stake
    vexDelay(500);               // wait for stake to be clamped
    roller_spin(100);            // wait for stake to be clamped
}

void auton(void)
{
    red_positive();
}

void skill(void)
{
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
