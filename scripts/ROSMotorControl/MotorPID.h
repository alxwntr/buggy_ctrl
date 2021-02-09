#ifndef MOTORPID_H
#define MOTORPID_H

#include <geometry_msgs/Twist.h>

#include "Encoder.h"
// Demand limits - SI units now in use
const int linDmdMax = 2;
const int angDmdMax = 2;

const float dT = 0.02; //50Hz - can't go too high because of lack of encoder pulses

//-------------------
// MotorController Class
//-------------------

class MotorController {
  public:
    const int motorNum;
    const bool  RHS;

    MotorController (const int motorNum, bool rhs, int encA, int encB, int motorA, int motorB)
      : motorNum(motorNum), RHS(rhs), encoder_(encA, encB), motorA_(motorA), motorB_(motorB)
    { }

    void setup_pins (void(*isr)(void));
    void process_pid (const geometry_msgs::Twist &twist);

    // XXX These should not forward through the MotorController
    void handle_irq() {
        encoder_.handle_irq();
    }
    float revolutions() {
        return encoder_.revolutions();
    }

  private:
    Encoder     encoder_;

    const int motorA_;
    const int motorB_;

    float lastError_ = 0.0, errorSum_ = 0.0;

    void        coast();
    Direction   find_direction(float demand);
    int         find_pwm(float demand, float speed);
    void        write_to_pins(Direction dir, int pwm);
};

#endif
