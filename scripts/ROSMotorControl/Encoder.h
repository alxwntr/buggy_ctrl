#ifndef _Encoder_h
#define _Encoder_h

#include "ring_buffer.h"

class Encoder {
    public:
    Encoder (int pinA, int pinB) : 
         /* Encoder signal pins. Pin A has an interrupt; pin B is used
          * only for quadrature. */
        pinA(pinA), pinB(pinB),
        edgesPerRev(6.0f), /* number of edges in one encoder revolution. Currently six. */
        spdTimeout(50000ul) /* spdTimeout allows spd to reach zero after a set period */
        { }

    void    setup_pins  (void(*isr)(void));
    void    handle_irq  ();

    float   speed       ();
    // This returns the number of revolutions since last time it was called
    float   revolutions    ();

#ifndef _Encoder_TESTING
    private:
#endif
    typedef unsigned long   time;

    enum Direction {
        Forward     = 1,
        Backward    = -1,
        Stopped     = 0,
    };

    const int               pinA;
    const int               pinB;

    const float             edgesPerRev;
    const time              spdTimeout;

    ring_buffer<time, 5>    tick_times;
    int32_t                 tick_count;
    int8_t                  tick_dir;

    void    set_tick_dir    (Direction dir);
};

#endif
