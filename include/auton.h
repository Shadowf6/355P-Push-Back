#pragma once

#include "api.h"
#include "intake.h"
#include "config.h"
#include "tracking/utils.h"
#include "lemlib/api.hpp"

extern lemlib::Drivetrain drivetrain;
extern pros::Imu imu;
extern pros::Rotation horizontalRotation;
extern lemlib::TrackingWheel horizontalWheel;
extern lemlib::Chassis chassis;
extern pros::adi::Pneumatics tongue;
extern pros::adi::Pneumatics bump;
extern pros::adi::Pneumatics lift;
extern pros::Distance left;
extern pros::Distance right;
extern int state;

void rightWing();
void leftWing();
void right4();
void left4();
void right6();
void left6();
void rightSplit();
void leftSplit();
void awpWing();
void autonSkills();
