#pragma once

#include "config.h"
#include "liblvgl/lvgl.h"
#include <iostream>

void nextAuton(lv_event_t* e);
void prevAuton(lv_event_t* e);
void createDisplay(lv_obj_t* screen);
void updateStatus(bool r, bool i, bool d, bool m);
void updateCoords(float x, float y, float t, float l, float r, float b);