#pragma once

#include "utils.h"
#include "config.h"
#include "odom.h"
#include "dist.h"

#include "lemlib/chassis/chassis.hpp"
#include "pros/distance.hpp"
#include <random>

struct particle { 
    float x;
    float y;
    float weight; 
};

static std::ranlux24_base rng(355);
static std::vector<particle> particles;

inline std::uniform_real_distribution<float> xDist(bounds[0], bounds[1]);
inline std::uniform_real_distribution<float> yDist(bounds[2], bounds[3]);

void makeParticles(int M);
void motionUpdate(std::vector<float> &pose, float dF);
std::vector<float> sensorUpdate(lemlib::Chassis *odom, pros::Distance *distX, pros::Distance *distY, pros::Imu *imu, std::vector<float> &init);
void weighParticles(std::vector<float> &pose);
void resample(int M);
void mcl(lemlib::Chassis *odom, pros::Distance *distX, pros::Distance *distY, pros::Imu *imu, std::vector<float> &init, float dF, int M);
