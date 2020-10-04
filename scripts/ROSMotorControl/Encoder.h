#ifndef _Encoder_h
#define _Encoder_h

#include "ring_buffer.h"

class Encoder {
    public:
    Encoder (int pinA) : 
        pinA(pinA), /* Encoder signal pin. Could be augmented with a second for quadrature at a later date. */
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

    const int               pinA;
    //const int             pinB;   // not yet

    const float             edgesPerRev;

    ring_buffer<time, 5>    tick_times;
    int32_t                 tick_count;

    const time              spdTimeout;
};

#endif
