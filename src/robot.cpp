#include "robot.hpp"
#include <iostream>
#include <cmath>
#include <vector>

// global variables
uint8_t exec = Exec::Drive | Exec::Auton;
Config cfg = {.rev = false, .rol = false, .tur = false, .brk = false};
Robot robo_g = {
    .base = {
        vex::motor(1, vex::gearSetting::ratio18_1, true),  // LF
        vex::motor(6, vex::gearSetting::ratio18_1, true),  // LM
        vex::motor(9, vex::gearSetting::ratio18_1, true),  // LB
        vex::motor(4, vex::gearSetting::ratio18_1, false), // RF
        vex::motor(7, vex::gearSetting::ratio18_1, false), // RM
        vex::motor(8, vex::gearSetting::ratio18_1, false), // RB
    },
    .intk = {
        vex::motor(5, vex::gearSetting::ratio18_1, true),
    },
    .pnch = {
        vex::motor(11, vex::gearSetting::ratio18_1, true),
    },
    .lady = {
        vex::motor(17, vex::gearSetting::ratio18_1, true),
    },
    .screen = vex::controller::lcd(),
    .mogo = vex::digital_out(vex::brain().ThreeWirePort.B),
    .dink = vex::digital_out(vex::brain().ThreeWirePort.D),
    .intkp = vex::digital_out(vex::brain().ThreeWirePort.C),
    .gyro = vex::inertial(4),
    .rotation = vex::rotation(15),
};
