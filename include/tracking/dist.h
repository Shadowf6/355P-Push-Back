#pragma once

#include "utils.h"
#include "pros/distance.hpp"
#include <cmath>

class Distance {
    private:
        pros::Distance *l;
        pros::Distance *r;
        pros::Distance *b;

        float initX = 0.0f;
        float initY = 0.0f;

        float voffsetL = 0.0f, hoffsetL = 0.0f;
        float voffsetR = 0.0f, hoffsetR = 0.0f;
        float voffsetB = 0.0f, hoffsetB = 0.0f;
    
    public:
        Distance(pros::Distance *left, pros::Distance *right, pros::Distance *bottom) :
        l(left), r(right), b(bottom) {}

        void setInit() {
            auto init = getDist(0);

            initX = init.first;

            if (init.second == 9999.0f) initY = b->get();
            else initY = init.second;
        }

        std::pair<float, float> getDist(float theta) {
            float x = 9999.0f, y = 9999.0f;

            float sinT = sinf(rad(theta));
            float cosT = cosf(rad(theta));

            float dl = l->get();
            float dr = r->get();
            float db = b->get();

            if (dl <= 2000) {
                x = dl - hoffsetL * cosT + voffsetL * sinT;
                x = inch(x) - initX;
            } else if (dr <= 2000) {
                x = 3657.6 - dr - hoffsetR * cosT + voffsetR * sinT;
                x = inch(x) - initX;
            }

            if (db <= 2000) {
                y = db - hoffsetB * sinT - voffsetB * cosT;
                y = inch(y) - initY;
            }

            return {x, y};
        }
};
