#ifndef MOTORARRAY_H
#define MOTORARRAY_H

#include <array>

#include "MotorPID.h"

typedef std::array<MotorController, 2> MotorArray;
extern MotorArray motors;

void setup_pins();

#endif
