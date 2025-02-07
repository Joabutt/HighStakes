#ifndef HELPER_HPP
#define HELPER_HPP

#include "robot.hpp"

void roller_spin(double velocity = 100);
void roller_spin(double velocity, double time, vex::timeUnits units,
                 bool block = true);
void mogo(bool open);
void dink(bool open);
void intkp(bool open);

#endif
