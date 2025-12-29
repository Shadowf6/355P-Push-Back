#pragma once

#include "config.h"
#include "liblvgl/lvgl.h"

void nextAuton(lv_event_t* e);
void prevAuton(lv_event_t* e);
void createDisplay(lv_obj_t* screen);
void updateCoords(float x, float y, float t);
void updateDist(float l, float r, float b);