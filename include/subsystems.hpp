#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

inline pros::MotorGroup lift({15,16}); // PT&&: check if need to change one lift to negative.
// port 15 connect ot top motor, port 16 connect to bottom motor of the lift
// 15 seems to just lift it up, but it always settles lopsided to teh "right side"

inline ez::Piston claw('A'); // the port that connects to the claw pneuma is port 'A'
//port on the back
inline ez::Piston toggle('B'); // PT&: toggle is to close claw

//inline pros::MotorGroup intake({-7,8});
// inline pros::adi::DigitalIn limit_switch('A');
