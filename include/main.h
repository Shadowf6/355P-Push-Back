#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_

#include "api.h"
#include "config.h"
#include "screen.h"
//#include "mcl/mcl.h"
//#include "mcl/odom.h"
//#include "mcl/dist.h"

#include "lemlib/api.hpp"
#include "liblvgl/lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif
void autonomous(void);
void initialize(void);
void disabled(void);
void competition_initialize(void);
void opcontrol(void);
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
#endif

#endif
 