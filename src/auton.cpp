#include "auton.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <cstdint>
#include "helper.hpp"
#include "robot.hpp"

void red_negative(void)
{
    move(630, 40, 40, true);    // move to stake
    vexDelay(300);              // wait for clamped
    mogo(true);                 // clamp stake
    roller_spin(-100);          // intake starting donut
    vexDelay(600);              // wait for donut to be intaken
    move(240, 20, -20, false);  // turn to red understacked donut at white line
    move(420, 70, 50, false);   // move to red understacked donut at white line & intake
    move(470, -55, -10, false); // move back
    roller_spin(0);
    intkp(true);                // lift intake
    vexDelay(300);             // wait
    intkp(true);                // push down intake
    roller_spin(-100);
    move(500, 30, 18);          // move to red understacked donut & intake
    move(90, 50, -50, false);   // turn to fourth donut
    move(120, 40, 40, false);    // move to fourth donut
    move(130, 30, 30, false);   // intake to fourth donut
    move(380, 20, -20, false);  // turn to red overstacked donut at alliance stake
    vexDelay(400);              // wait
    intkp(true);                // lift intake
    move(960, 50, 50, false);   // move to red overstacked donut at alliance stake & intake
    vexDelay(600);              // wait for donut to be intaken
    intkp(true);                // press down intake
    vexDelay(700);              // wait
    move(200, 50, 50, true);    // Move back
}

// Autonomous route for Red Negative
void red_positive(void)
{
    lady(2);                     // ladybrown alliance donut
    move(730, -60, -40, false);  // turn to stake
    move(150, 50, 50, false);    // move to stake
    vexDelay(100);               // wait for robot to reach stake
    mogo(true);                  // clamp stake
    lady(0);                     // retract ladybrown
    vexDelay(500);               // wait for stake to be clamped
    roller_spin(-100);           // intake starting donut
    move(200, -20, 20, false);    // turn to red understacked donut
    move(290, 50, 50, false);    // move to red understacked donut & intake
    vexDelay(600);               // wait for donut to be intaken
    move(200, -20, 20, false);   // turn to red overstacked donut at white line
    intkp(true);                 // lift intake
    move(960, 50, 50, false);    // move to red overstacked donut at white line
    vexDelay(600);               // wait
    intkp(true);                 // press down intake
    move(100, 50, 50, true);     // move back and intake
    move(310, -50, 50, false);   // turn to positive corner
    move(650, 50, 50, false);    // move to positive corner
    dink(true);                  // open dinkler
    vexDelay(700);               // wait
    move(200, 80, -80, false);   // clear positive corner
}

// Autonomous route for Blue Positive
void blue_negative(void)
{
    move(630, 40, 40, true);    // move to stake
    vexDelay(300);              // wait for clamped
    mogo(true);                 // clamp stake
    roller_spin(-100);          // intake starting donut
    vexDelay(600);              // wait for donut to be intaken
    move(240, -20, 20, false);  // turn to red understacked donut at white line
    move(390, 50, 70, false);   // move to red understacked donut at white line & intake
    move(440, -10, -55, false); // move back
    roller_spin(0);
    intkp(true);                // lift intake
    vexDelay(300);             // wait
    intkp(true);                // push down intake
    roller_spin(-100);
    move(500, 18, 30);          // move to red understacked donut & intake
    move(90, -50, 50, false);   // turn to fourth donut
    move(120, 40, 40, false);    // move to fourth donut
    move(130, 30, 30, false);   // intake to fourth donut
    move(380, -20, 20, false);  // turn to red overstacked donut at alliance stake
    vexDelay(400);              // wait
    intkp(true);                // lift intake
    move(960, 50, 50, false);   // move to red overstacked donut at alliance stake & intake
    vexDelay(600);              // wait for donut to be intaken
    intkp(true);                // press down intake
    vexDelay(700);              // wait
    move(200, 50, 50, true);    // Move back
}

// Autonomous route for Blue Negative
void blue_positive(void)
{
    lady(2);                    // ladybrown alliance donut
    move(630, -30, -50, false); // move to stake
    vexDelay(100);              // wait for robot to reach stake
    mogo(true);                 // clamp stake
    lady(0);                    // retract ladybrown
    vexDelay(500);              // wait for stake to be clamped
    roller_spin(-100);          // intake starting donut
    move(290, 50, 20, false);   // move to red understacked donut & intake
    vexDelay(600);              // wait for donut to be intaken
    move(290, -20, 20, false);  // turn to red overstacked donut at white line
    intkp(true);                // lift intake
    move(960, 50, 50, false);   // move to red overstacked donut at white line
    vexDelay(600);              // wait
    intkp(true);                // press down intake
    move(100, 50, 50, true);    // move back and intake
    move(310,-50, 50, false);   // turn to positive corner
    move(650, 50, 50, false);   // move to positive corner
    dink(true);                 // open dinkler
    vexDelay(700);              // wait
    move(200, -80, 80, false);  // clear positive corner
}

void auton(void)
{
    blue_negative();
}

void skill(void)
{
    // Implement your skills logic here
}

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

void move(double degree, double left, double right, bool invert)
{
    if (invert)
    {
        double temp = left;
        left = -right;
        right = -temp;
    }

    if (cfg.rev)
    {
        left = -left;
        right = -right;
    }

    if (degree < 0)
    {
        degree = -degree;
        left = -left;
        right = -right;
    }

    // Reset encoders and gyro
    for (uint8_t i = 0; i < MOTORS_BASE; i++)
    {
        robo_g.base[i].resetPosition();
    }
    robo_g.gyro.resetRotation(); // Reset gyro

    vex::timer moveTimer;
    moveTimer.clear();

    // PID Constants
    double Kp = 0.5;  // Proportional constant
    double Ki = 0.01; // Integral constant
    double error, integral = 0;

    while (std::fabs(robo_g.base[LF].position(vex::rotationUnits::deg)) < degree)
    {
        double headingError = robo_g.gyro.rotation(vex::rotationUnits::deg);
        integral += headingError;                                  // Accumulate integral error
        double correction = (Kp * headingError) + (Ki * integral); // PID Correction

        for (uint8_t i = 0; i < MOTORS_BASE; i++)
        {
            double motorSpeed = (i < drive_motors::RF) ? left : right;
            if (i < drive_motors::RF)
            {
                motorSpeed -= correction; // Adjust left motors
            }
            else
            {
                motorSpeed += correction; // Adjust right motors
            }
            robo_g.base[i].spin(vex::directionType::fwd, motorSpeed, vex::percentUnits::pct);
        }

        vex::task::sleep(25);
    }

    // Stop motors
    for (uint8_t i = 0; i < MOTORS_BASE; i++)
    {
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

// Define stages for the "lady" motor
std::vector<double> stages1 = {30, -60, -300}; // Rotation values for each stage
uint8_t currentStage1 = 0;                     // Start at stage 0

void lady(double stage)
{
    if (stage >= stages1.size())
    {
        return; // Prevent invalid stage access
    }

    double targetRotation = stages1[stage];
    double currentRotation = robo_g.lady[0].position(vex::rotationUnits::deg);

    // Determine direction and velocity
    double direction = targetRotation > currentRotation ? 1 : -1;
    double velocity = direction * 50; // Adjust speed as needed

    // Move the motor to the target stage
    robo_g.lady[0].spinToPosition(targetRotation, vex::rotationUnits::deg, velocity, vex::velocityUnits::pct, true);

    currentStage1 = stage; // Update current stage
    if (currentStage1 == 0)
    {
        robo_g.lady[0].resetPosition(); // Reset the rotation sensor when reaching stage 0
    }
}