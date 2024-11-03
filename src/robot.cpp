#include "robot.hpp"
#include <iostream>
#include <cmath>

// global variables
uint8_t exec = Exec::Drive | Exec::Auton;
Config cfg = {.rev = false, .rol = false, .tur = false, .brk = false};
Robot robo_g = {
    .base = {
        vex::motor(1, vex::gearSetting::ratio18_1, true),   // LF
        vex::motor(6, vex::gearSetting::ratio18_1, true),   // LM
        vex::motor(9, vex::gearSetting::ratio18_1, true),   // LB
        vex::motor(0, vex::gearSetting::ratio18_1, false),  // RF
        vex::motor(7, vex::gearSetting::ratio18_1, false),  // RM
        vex::motor(8, vex::gearSetting::ratio18_1, false),  // RB
    },
    .intk = {
        vex::motor(2, vex::gearSetting::ratio18_1, true),
        vex::motor(3, vex::gearSetting::ratio18_1, false),
    },
    .pnch = {
        vex::motor(11, vex::gearSetting::ratio18_1, true),
    },
    .screen = vex::controller::lcd(),
    .wing = vex::digital_out(vex::brain().ThreeWirePort.A),
    .hang = vex::digital_out(vex::brain().ThreeWirePort.F),
    .gyro = vex::inertial(18),
    .leftRtn = vex::rotation(17),
    .horizRtn = vex::rotation(19),
};



