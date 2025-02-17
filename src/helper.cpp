#include "helper.hpp"

#include <cmath>
#include <iostream>

#include "robot.hpp"

void roller_spin(double velocity) {
    for (uint8_t i = 0; i < MOTORS_INTK; i++)
        robo_g.intk[i].spin(vex::directionType::fwd, velocity, vex::velocityUnits::pct);

    for (uint8_t i = 0; i < MOTORS_INTK; i++)
        std::cout << std::boolalpha << robo_g.intk[i].isSpinning() << std::endl;
}

static double _vel, _tim;

static int _roller_spin(void) {
    roller_spin(_vel, _tim, vex::timeUnits::msec, true);
    return 0;
}

void roller_spin(double velocity, double time, vex::timeUnits units,
                 bool waitForCompletion) {
    _vel = velocity;
    _tim = time * (units == vex::timeUnits::sec
                                                 ? 1000
                                                 : 1);

    if (!waitForCompletion) {
        vex::task _ = vex::task(_roller_spin);
        return;
    }

    for (uint8_t i = 0; i < MOTORS_INTK; i++)
        robo_g.intk[i].spinFor(time, units, velocity, vex::velocityUnits::pct);
}


void mogo(bool open) {
    robo_g.mogo.set(robo_g.mogo.value() ^ 1);
}

void dink(bool open){
    robo_g.dink.set(robo_g.dink.value() ^ 1);
}

void intkp(bool open){
    robo_g.intkp.set(robo_g.intkp.value() ^ 1);
}

