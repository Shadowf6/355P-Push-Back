#pragma once

#include "config.h"
#include "liblvgl/lvgl.h"
#include <iostream>

void createDisplay(lv_obj_t* screen);
void setStatus(bool r, bool i, bool d, bool m);
void updateCoords(float x, float y, float t, float l, float r, float f, float b);
