#ifndef DEBUG_H
#define DEBUG_H

#include <Arduino.h>
#include <ros.h>
#include <buggy_ctrl/pidInfo.h>
#include <buggy_ctrl/ctrlDebug.h>

extern buggy_ctrl::pidInfo pidInfo[2];
//int16 pwm
//float32 speed
//float32 error
//int8 direction

extern buggy_ctrl::ctrlDebug debugInfo;

void publish_debug( ros::Publisher& pub );

#endif
