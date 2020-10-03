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

    Odometer (float wheelbase, float wheelDia, float gearboxRatio)
      : wheelbase_(wheelbase), wheelDia_(wheelDia), gearboxRatio_(gearboxRatio)
    { }
    
    void
    calculate_moves();

   private:
    const float wheelbase_;
    const float wheelDia_;
    const float gearboxRatio_;
};
extern Odometer odometer;

#endif
