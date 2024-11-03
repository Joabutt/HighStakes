#include "auton.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include "helper.hpp"
#include "robot.hpp"

double pid_control(double target, double current, double &integral, double &previous_error, double kp, double ki, double kd) {
    double error = target - current;
    integral += error;
    double derivative = error - previous_error;
    previous_error = error;
    return (kp * error) + (ki * integral) + (kd * derivative);
}

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

void move(double target_distance, double kp, double ki, double kd) {
    double integral = 0.0;
    double previous_error = 0.0;
    double current_distance = 0.0;
    double tolerance = 0.5;

    while (fabs(target_distance - current_distance) > tolerance) {
        current_distance = get_current_distance();  // Replace with actual distance reading
        double power = pid_control(target_distance, current_distance, integral, previous_error, kp, ki, kd);
        set_motor_power(power);  // Replace with actual motor control function
        vex::task::sleep(20);
    }
    set_motor_power(0);  // Stop motor
}

void turn_until(double target_angle, double kp, double ki, double kd) {
    double integral = 0.0;
    double previous_error = 0.0;
    double current_angle = 0.0;
    double tolerance = 1.0;

    while (fabs(target_angle - current_angle) > tolerance) {
        current_angle = get_current_angle();  // Replace with actual angle reading
        double turn_power = pid_control(target_angle, current_angle, integral, previous_error, kp, ki, kd);
        set_turn_power(turn_power);  // Replace with actual turning motor control
        vex::task::sleep(20);
    }
    set_turn_power(0);  // Stop turning
}
