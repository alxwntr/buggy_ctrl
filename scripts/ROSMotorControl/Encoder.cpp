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

    if (micros() - last > spdTimeout) {
        noInterrupts(); tick_times.clear(); interrupts();
        return 0.0;
    }

    time    interval    = last - first;
    if (interval == 0)
        return 0.0;

    float   speed       = (count / edgesPerRev) / (float(interval) / 1000000); // Gives revs per sec for the encoder
    return speed;
}

float
Encoder::revolutions()
{
    noInterrupts();
    int32_t ticks   = tick_count;
    tick_count      = 0;
    interrupts();

    return float(ticks) / edgesPerRev;
}
