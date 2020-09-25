#ifndef MOTORPID_H
#define MOTORPID_H

#include <geometry_msgs/Twist.h>

const int linDmdMax = 200;
const int angDmdMax = 100;

const float dT = 0.02; //50Hz - can't go too high because of lack of encoder pulses

class MotorController {
  public:
    const bool  RHS;
    // -1 = bkwrds, 0 = static, 1 = frwrds (need to think how to sort this out)
    int         dir = 0;

    volatile float encCount = 0.0;

    MotorController (bool rhs, int encPin, int motorA, int motorB)
      : RHS(rhs), encPin_(encPin), motorA_(motorA), motorB_(motorB)
    { }

    void setup_pins (void(*isr)(void));
    void process_pid (const geometry_msgs::Twist &twist);
    void handle_irq ();

  private:
    const int encPin_;
    const int motorA_;
    const int motorB_;

    volatile unsigned long lastTime_ = 0;
    volatile double spd_ = 0;
    volatile int count_ = 0;

    float lastError_ = 0.0, errorSum_ = 0.0;

    int drivePin_, gndPin_;
};

#endif
