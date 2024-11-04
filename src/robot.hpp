#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "vex.h"

typedef enum { LF, LM, LB, RF, RM, RB, MAX } drive_motors;

#define MOTORS_INTK 1
#define MOTORS_PNCH 1
#define MOTORS_BASE drive_motors::MAX

/// A structure to group hardware of the robot.
struct Robot {
    /// Remote-controller for VEX.
    vex::controller ctrl;
    /// Drivetrain, represented as an array of motors.
    vex::motor base[MOTORS_BASE];
    /// Motor for intake roller mechanism.
    vex::motor intk[MOTORS_INTK];
    /// Motor for punching mechanism.
    vex::motor pnch[MOTORS_PNCH];
    /// LCD object for controller.
    vex::controller::lcd screen;
    /// DO object for wing.
    vex::digital_out wing;
    // DO object for hang.
    vex::digital_out hang;
    // Inertial sensor.
    vex::inertial gyro;
};

#define SPEED 1

struct Config {
    bool rev : 1; /**< Whether the controls are reversed. */
    bool tur : 1; /**< Whether the turbo mode is enabled. */
    bool brk : 1; /**< Whether the brake mode is enabled. */
    bool rol : 1; /**< Whether the roller is spinning. */
};

enum Exec {
    Drive = 1 << 0,
    Auton = 1 << 1,
    Skill = 1 << 2,
};

// global variables
extern Config cfg;
extern Robot robo_g; /**< global `Robo` variable */
extern uint8_t exec; /** parts of program to execute */

#endif