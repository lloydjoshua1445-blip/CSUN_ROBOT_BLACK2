#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

inline pros::MotorGroup lift({15,16});
inline ez::Piston claw('A');
inline ez::Piston toggle('B');

//inline pros::MotorGroup intake({-7,8});
// inline pros::adi::DigitalIn limit_switch('A');