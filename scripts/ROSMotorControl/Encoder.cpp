#include <Arduino.h>

#include "Encoder.h"

void
Encoder::setup_pins(void(*isr)(void))
{
    pinMode(pinA, INPUT); 
    attachInterrupt(digitalPinToInterrupt(pinA), isr, CHANGE);
}

void 
Encoder::handle_irq ()
{
    time    now     = micros();

    // XXX check direction
    tick_count++;

    tick_times.push(now);
}

float
Encoder::speed ()
{
    noInterrupts();
    time    last    = tick_times.last();
    time    first   = tick_times.first();
    int32_t count   = tick_times.count();
    interrupts();

    if (count == 0)
        return 0.0;

    if (micros() - last > 50000) {
        noInterrupts(); tick_times.clear(); interrupts();
        return 0.0;
    }

    float   interval    = float(last - first);
    float   speed       = (speed_factor * count) / interval;

    return speed;
}

float
Encoder::distance()
{
    noInterrupts();
    int32_t ticks   = tick_count;
    tick_count      = 0;
    interrupts();

    return distance_factor * float(ticks);
}
