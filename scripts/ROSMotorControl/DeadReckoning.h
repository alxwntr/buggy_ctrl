#ifndef DEADRECKONING_H
#define DEADRECKONING_H

#include <Arduino.h>

class Odometer {
  public:
    //Position variables:
    float x = 0.0;
    float y = 0.0;
    float theta = 0.0;
    float debug;
    
    void
    calculate_moves();
};
extern Odometer odometer;

#endif
