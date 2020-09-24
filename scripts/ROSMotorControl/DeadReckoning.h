#ifndef DEADRECKONING_H
#define DEADRECKONING_H

#include <Arduino.h>

#include "MotorPID.h"

const int num_motors = 4;

extern MotorController motors[num_motors];

void calculate_moves(float &x, float &y, float &theta);
void setup_motors();

#endif
