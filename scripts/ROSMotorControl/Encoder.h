#ifndef _Encoder_h
#define _Encoder_h

#include "ring_buffer.h"

class Encoder {
    public:
    Encoder (int pinA) : 
        pinA(pinA),
        /* The previous code in MotorPID was dividing the time for three
         * ticks into 1e6. This attempts to produce the same result, given
         * that the number of valid ticks we have will vary. This needs
         * revising. */
        speed_factor(333333.0f),
        /* For now set the distance factor to 1. This should be
         * calculated based on wheel radius etc. */
        distance_factor(1.0f)
        { }

    void    setup_pins  (void(*isr)(void));
    void    handle_irq  ();

    float   speed       ();
    // This returns the distance travelled since last time it was called
    float   distance    ();

    private:
    typedef unsigned long   time;

    const int               pinA;
    //const int             pinB;   // not yet

    const float             speed_factor;
    const float             distance_factor;

    ring_buffer<time, 5>    tick_times;
    int32_t                 tick_count;
};

#endif
