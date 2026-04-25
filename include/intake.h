#pragma once

#include "config.h"
#include "pros/motors.hpp"
#include "pros/adi.hpp"

inline pros::Motor intake(-19, pros::MotorGearset::blue); 
inline pros::Motor score(7, pros::MotorGearset::red, pros::MotorUnits::degrees);
inline pros::adi::Pneumatics hood('D', false);

constexpr float kP = 10;

constexpr float starting = 0;
constexpr float longgoal = 175;
constexpr float midgoal = 185;

void in();
void out();
float stickUp();
float stickMid();
float stickDown();
