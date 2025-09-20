#pragma once

#include "utils.h"

#include "pros/distance.hpp"

#include <cmath>

class Distance {
    private:
        pros::Distance *l, *r, *b, *t;

        float offsetL = 0.0f;
        float offsetR = 0.0f;
        float offsetB = 0.0f;
        float offsetT = 0.0f;

        float initX = 0.0f;
        float initY = 0.0f;
        float initT = 0.0f;
    
    public:
        Distance(pros::Distance *left, pros::Distance *right, pros::Distance *bottom, pros::Distance *top) {
            l = left;
            r = right;
            b = bottom;
            t = top;
        }

        float updateX();

        float updateY();

        float getInit();

        float updateInit();
};
