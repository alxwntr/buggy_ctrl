#include <Arduino.h>

#include "Encoder.h"

void
Encoder::setup_pins()
{
    pinMode(pinA, INPUT); 
    pinMode(pinB, INPUT);
}

void 
Encoder::handle_irq (InterruptPin pin)
{
    time    now     = micros();
    int     a       = digitalRead(pinA);
    int     b       = digitalRead(pinB);

    Direction   dir;
    
    if (pin == PinAInterrupt) {
        dir = ((a != b) ? Forward : Backward);
    }
    else {
        dir = ((a == b) ? Forward : Backward);
    }

    set_tick_dir(dir);

    if (dir == Forward)
        tick_count++;
    else
        tick_count--;
    
    tick_times.push(now);
}

/* Must be called with interrupts disabled */
void
Encoder::set_tick_dir(Direction dir)
{
    /* We could check to see if we don't need to call clear(), but let's
     * assume the STL can make that check as quickly as we can. */
    if (tick_dir != dir) tick_times.clear();
    tick_dir = dir;
}

float
Encoder::speed ()
{
    noInterrupts();
    time        last    = tick_times.last();
    time        first   = tick_times.first();
    int32_t     count   = tick_times.count();
    Direction   dir     = tick_dir;
    interrupts();

    if (count == 0)
        return 0.0;

    if (micros() - last > spdTimeout) {
        noInterrupts(); set_tick_dir(Stopped); interrupts();
        return 0.0;
    }

    time    interval    = last - first;
    if (interval == 0)
        return 0.0;

    float   edgesPerUs   = count / float(interval);
    float   revsPerSec   = (edgesPerUs * 1000000) / edgesPerRev;

    return (dir == Forward) ? revsPerSec : -revsPerSec;
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
