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
        float initT = 90.0f;
    
    public:
        Distance(pros::Distance *left, pros::Distance *right, pros::Distance *bottom, pros::Distance *top) :
        l(left), r(right), b(bottom), t(top) {}

        void updateInit(float x, float y, float theta) {
            initX = x;
            initY = y;
            initT = theta;
        }

        float* calcX(float theta) {
            float x = 0.0f;
            float* ptr = nullptr;

            bool leftExists = (l != nullptr);
            bool rightExists = (r != nullptr);
            bool leftValid = false;
            bool rightValid = false;

            float ld, rd;

            if (leftExists) {
                leftValid = (l->get_distance() <= 200);
                ld = inch(l->get_distance()) + offsetL;
            }

            if (rightExists) {
                rightValid = (r->get_distance() <= 200);
                rd = inch(r->get_distance()) + offsetR;
            } 

            if (leftExists && leftValid) { x = cosf(rad(theta)) * ld - initX; ptr = &x; }
            else if (rightExists && rightValid) { x = 144 - (cosf(rad(theta)) * rd - initX); ptr = &x; }

            if (leftExists && leftValid && rightExists && rightValid) {
                float w = l->get_confidence() + r->get_confidence();
                x = (l->get_confidence() / w) * x + (r->get_confidence() / w) * (144 - (cosf(rad(theta)) * rd - initX));
            }

            return ptr;
        }

        float* calcY(float theta) {
            float y = 0.0f;
            float* ptr = nullptr;

            bool botExists = (b != nullptr);
            bool topExists = (t != nullptr);
            bool botValid = false;
            bool topValid = false;

            float bd, td;

            if (botExists) {
                botValid = (b->get_distance() <= 200);
                bd = inch(b->get_distance()) + offsetB;
            }
            
            if (topExists) {
                topValid = (t->get_distance() <= 200);
                td = inch(t->get_distance()) + offsetT;
            }

            if (botExists && botValid) { y = cosf(rad(theta)) * bd - initY; ptr = &y; }
            else if (topExists && topValid) { y = 144 - (cosf(rad(theta)) * td - initY); ptr = &y; }

            if (botExists && botValid && topExists && topValid) {
                float w = b->get_confidence() + r->get_confidence() ;
                y = (b->get_confidence() / w) * y + (t->get_confidence() / w) * (144 - (cosf(rad(theta)) * td - initY));
            }

            return ptr;
        }

        std::pair<float, float> poseReset(float theta) {
            if (calcX(theta) != nullptr && calcY(theta) != nullptr) {
                float x = *calcX(theta);
                float y = *calcY(theta);
                float c = cosf(rad(theta));
                float s = sinf(rad(theta));
                
                return {x, y};
            }

            return {676741, 676741};
        }
};
