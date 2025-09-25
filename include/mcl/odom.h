#pragma once

#include "utils.h"

#include "pros/imu.hpp"
#include "lemlib/chassis/trackingWheel.hpp"

#include <cmath>

struct pose {
    float x;
    float y;
    float theta;
};

class Odometry {
    private:
        pros::Imu *imu;
        lemlib::TrackingWheel *vert;
        lemlib::TrackingWheel *hori;

        pose p {0.0f, 0.0f, 0.0f};

        float offsetX = -4.5f;
        float offsetY = -0.5f;

        float lastX = 0.0f;
        float lastY = 0.0f;
        float lastT = 0.0f;
    
    public:
        Odometry(pros::Imu *inertial, lemlib::TrackingWheel *vertical, lemlib::TrackingWheel *horizontal) : 
        imu(inertial), vert(vertical), hori(horizontal) {}

        void update() {
            p.theta = wrap((float)imu->get_heading());
            float dt = p.theta - lastT;

            float dx = hori->getDistanceTraveled() - lastX - offsetX * dt;
            float dy = vert->getDistanceTraveled() - lastY - offsetY * dt;
            lastX = hori->getDistanceTraveled();
            lastY = vert->getDistanceTraveled();
            lastT = p.theta;

            float sinT = sinf(rad(p.theta));
            float cosT = cosf(rad(p.theta));

            p.x += dx * cosT + dy * sinT;
            p.y += -dx * sinT + dy * cosT;
        }

        pose getPose() {
            return p;
        }
};
