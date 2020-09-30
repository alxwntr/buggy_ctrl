//INCLUDE: -Imock
//SOURCES: ../Encoder.cpp

#define _Encoder_TESTING

#include <string>

#include "tap.h"
#include "Arduino.h"
#include "Encoder.h"

using namespace std::literals;
using namespace tap::exports;

void mock_isr(void) { return; }

Encoder encoder { 76 };
auto &tcount    = encoder.tick_count;
auto &ttimes    = encoder.tick_times;

void 
check_constructor()
{
    is(encoder.pinA,            76,         "pinA init");
    is(encoder.speed_factor,    333333.0f,  "speed_factor init");
    is(encoder.distance_factor, 1.0f,       "distance_factor init");
    is(tcount,                  0,          "tick_count init");
    ok(ttimes.empty(),                      "tick_times init");
}

void
check_speed_distance(float speed, float distance, std::string msg)
{
    is(encoder.speed(),     speed,      "speed " + msg);
    is(encoder.distance(),  distance,   "distance " + msg);
}

int
main(int argc, char **argv)
{
    check_constructor();
    check_speed_distance(0, 0, "at start");

    clear_mock_results();
    encoder.setup_pins(mock_isr);

    mock_results_are({{
        {"pinMode",             {{76, INPUT}}},
        {"attachInterrupt",     {{76, (intptr_t)mock_isr, CHANGE}}},
    }},                                     "setup_pins");

    clear_mock_results();
    tap::set_micros(4200);
    encoder.handle_irq();

    mock_results_are({{ 
        {"micros", {}},
    }},                             "handle_irq calls micros once");
    is(tcount,              1,      "handle_irq incs tick_count");
    is(ttimes.count(),      1,      "handle_irq adds a time");
    is(ttimes.first(),      4200,   "handle_irq adds correct time");

    done_testing();
}
