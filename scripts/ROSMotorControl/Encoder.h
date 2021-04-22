#ifndef _Encoder_h
#define _Encoder_h

#include "ring_buffer.h"

enum Direction {
    Forward     = 1,
    Backward    = -1,
    Stopped     = 0,
};

enum InterruptPin {
    PinAInterrupt     = 1,
    PinBInterrupt    = -1,
};

class Encoder {
    public:
    Encoder (int pinA, int pinB) : 
         /* Encoder signal pins. Pin A has an interrupt; pin B is used
          * only for quadrature. */
        pinA(pinA), pinB(pinB),
        edgesPerRev(32.0f), /* number of edges in one encoder revolution. Currently 32 per channel. */
        spdTimeout(50000ul) /* spdTimeout allows spd to reach zero after a set period */
        { }

    void    setup_pins  ();
    void    handle_irq  (InterruptPin pin);

    float   speed       ();
    // This returns the number of revolutions since last time it was called
    float   revolutions    ();

#ifndef _Encoder_TESTING
    private:
#endif
    typedef unsigned long   time;

    const int               pinA;
    const int               pinB;

    const float             edgesPerRev;
    const time              spdTimeout;

    ring_buffer<time, 5>    tick_times;
    int32_t                 tick_count;
    Direction               tick_dir;

    void    set_tick_dir    (Direction dir);
};

#endif
