#include "intake.h"

void in() {
    intake.move(127);
}

void out() {
    intake.move(-127);
}

float stickUp() {
    float error = longgoal - score.get_position();
    score.move((int)(kP * error));
    return error;
}

float stickMid() {
    float error = midgoal - score.get_position();
    score.move((int)((driverControl ? 0.4f : 0.2f) * kP * error));
    return error;
}

float stickDown() {
    float error = starting - score.get_position();
    score.move((int)(kP * error));
    return error;
}
