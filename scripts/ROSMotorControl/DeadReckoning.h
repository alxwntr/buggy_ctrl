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

    Odometer (float wheelbase, float wheelDia)
      : wheelbase_(wheelbase), wheelDia_(wheelDia)
    { }
    
    void
    calculate_moves();

   private:
    const float wheelbase_;
    const float wheelDia_;
};
extern Odometer odometer;

#endif
