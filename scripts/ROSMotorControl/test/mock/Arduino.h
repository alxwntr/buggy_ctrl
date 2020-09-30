// This file need to define the same include guard macro as the real
// Arduino.h.
#ifndef Arduino_h
#define Arduino_h

#include "tap.h"

#include <vector>
#include <map>
#include <string>

#define     INPUT       0
#define     OUTPUT      1
#define     CHANGE      2

#define     digitalPinToInterrupt(x)    (x)

inline void
pinMode(uint32_t pin, uint32_t mode) {
    tap::mock_results.push_back({"pinMode", {{pin, mode}}});
}

inline void
attachInterrupt(uint32_t irq, void(*isr)(void), uint32_t mode) {
    tap::mock_results.push_back({"attachInterrupt", 
        {{irq, (intptr_t)isr, mode}}});
}

namespace tap {
    inline uint32_t micros_result = 0;

    inline void
    set_micros(uint32_t t) { 
        micros_result = t;
    }
}

inline uint32_t micros() {
    return tap::micros_result;
}

inline void
noInterrupts() {
    tap::mock_results.push_back({"noInterrupts", {}});
}

inline void
interrupts() {
    tap::mock_results.push_back({"interrupts", {}});
}

#endif
