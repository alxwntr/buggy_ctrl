#ifndef MOTORPID_H
#define MOTORPID_H

#include <geometry_msgs/Twist.h>

#include "Encoder.h"

const int linDmdMax = 200;
const int angDmdMax = 100;

const float dT = 0.02; //50Hz - can't go too high because of lack of encoder pulses

//-------------------
// MotorController Class
//-------------------

class MotorController {
  public:
    const bool  RHS;
    // -1 = bkwrds, 0 = static, 1 = frwrds (need to think how to sort this out)
    int         dir = 0;

    MotorController (bool rhs, int encPin, int motorA, int motorB)
      : RHS(rhs), encoder_(encPin), motorA_(motorA), motorB_(motorB)
    { }

    void setup_pins (void(*isr)(void));
    void process_pid (const geometry_msgs::Twist &twist);

    // XXX These should not forward through the MotorController
    void handle_irq() {
        encoder_.handle_irq();
    }
    float distance() {
        return encoder_.distance() * dir;
    }

  private:
    Encoder     encoder_;

    const int motorA_;
    const int motorB_;

    float lastError_ = 0.0, errorSum_ = 0.0;

    int drivePin_, gndPin_;
};

#endif
