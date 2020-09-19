#ifndef MOTORPID_H
#define MOTORPID_H

//-------------------------
//  PID variables
//-------------------------

const float dT = 0.02; //seconds
const float Kp = 0.5;
const float Ki = 10.0;
const float Kd = 0.01;

//-------------------------
//  Car hardware variables
//-------------------------

class MotorController {

    const int encPin_;
    const int motorA_;
    const int motorB_;

    volatile unsigned long lastTime_ = 0;
    volatile double spd_ = 0;
    volatile int count_ = 0;

    float lastError_ = 0.0, errorSum_ = 0.0;

  public:

    int dir = 0; // -1 = bkwrds, 0 = static, 1 = frwrds (need to think how to sort this out)
    bool braking = false;

    volatile float encCount = 0.0;
    float floorSpeed = 0.0; //not needed?

    MotorController (int encPin, int motorA, int motorB)
      : encPin_(encPin), motorA_(motorA), motorB_(motorB)
    { }

    void setup_pins (void(*isr)(void))
    {
      pinMode(motorA_, OUTPUT);
      //Set ground on one side of motor (for now)
      digitalWrite(motorB_, LOW);
      //Encoder pins
      pinMode(encPin_, INPUT);

      attachInterrupt(digitalPinToInterrupt(encPin_), isr, CHANGE);
    }

    //PID loop:
    void process_pid (int scaledDmd)
    {
      //Brake (if travelling in wrong direction)
      if (braking)
      {
        digitalWrite(motorA_, HIGH);
        digitalWrite(motorB_, HIGH);
        return;
      }

      //Temporary variables:
      int     pwm;
      int     scaledSpd = map(spd_, 0, 170, 0, 255);
      float   error;

      //Calculate error and set PWM level:
      error       = scaledDmd - scaledSpd;
      errorSum_   += error * dT;
      pwm         = Kp * error + Ki * errorSum_ + Kd * (error - lastError_) / dT;
      lastError_  = error;

      //Constrain PWM and ensure it gets to zero (immediately):
      if (scaledDmd == 0) pwm = 0;
      if (pwm < 0)        pwm = 0;
      if (pwm > 255)      pwm = 255;
      analogWrite(motorA_, pwm);

      //Allow spd_to reach zero if no motion
      if ((micros() - lastTime_) > 50000)
        spd_ = 0;
    }

    //Called from ISR#
    void handle_irq ()
    {
      unsigned long   timeNow;
      float           tickTime;

      //Do nothing for first 4 times:
      count_++;
      encCount++;
      if (count_ < 3) return;
      count_ = 0;

      //Set time variables and calculate speed:
      timeNow     = micros();
      tickTime    = float(timeNow - lastTime_);
      spd_        = 1000000 / tickTime; // gives Hz for 0.5 encoder revs. RECONSIDER
      lastTime_   = timeNow;
    }
};

#endif
