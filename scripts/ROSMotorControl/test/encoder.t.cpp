//INCLUDE: -Imock
//SOURCES: ../Encoder.cpp

#define _Encoder_TESTING

#include "tap.h"
#include "Arduino.h"
#include "Encoder.h"

using namespace tap::exports;

void mock_isr(void) { return; }

Encoder encoder { 76 };

int
main(int argc, char **argv)
{
    is(encoder.pinA,            76,         "pinA set correctly");
    is(encoder.speed_factor,    333333.0f,  "speed_factor set correctly");
    is(encoder.distance_factor, 1.0f,       "distance_factor set correctly");

    clear_mock_results();
    encoder.setup_pins(mock_isr);

    mock_results_are({{
        {"pinMode",             {{76, INPUT}}},
        {"attachInterrupt",     {{76, (intptr_t)mock_isr, CHANGE}}},
    }},                                     "setup_pins");

    done_testing();
}
