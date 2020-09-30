#define _ring_buffer_TESTING

#include "tap.h"
#include "ring_buffer.h"

using namespace tap::exports;

ring_buffer<unsigned long, 5> ticks;

void show_ticks () {
    diag() << "ticks.start " << ticks.start;
    diag() << "ticks.valid " << ticks.valid;

    log d = diag();
    d << "ticks contents: [";
    for (int i = 0; i < 5; i++) {
        d << ticks.buffer[i] << ", ";
    }
    d << "]";
}

int
main (int argc, char **argv)
{
    ok(ticks.empty(),           "ticks is empty at the start");
    is(ticks.count(),   0,      "ticks.count is 0");

    ticks.push(200);
    ok(!ticks.empty(),          "ticks is now not empty");
    is(ticks.count(),   1,      "ticks.count increases when not full");
    show_ticks();
    is(ticks.first(),   200ul,  "correct value for first");
    is(ticks.last(),    200ul,  "correct value for last");

    ticks.push(300);
    ok(!ticks.empty(),          "ticks is now not empty");
    is(ticks.count(),   2,      "ticks.count increases when not full");
    show_ticks();
    is(ticks.first(),   200ul,  "correct value for first");
    is(ticks.last(),    300ul,  "correct value for last");

    ticks.push(400); ticks.push(500); ticks.push(600);
    ok(!ticks.empty(),          "ticks is now not empty");
    is(ticks.count(),   5,      "ticks.count increases when not full");
    show_ticks();
    is(ticks.first(),   200ul,  "correct value for first");
    is(ticks.last(),    600ul,  "correct value for last");

    ticks.push(700);
    ok(!ticks.empty(),          "ticks is now not empty");
    is(ticks.count(),   5,      "ticks.count remains at 5");
    show_ticks();
    is(ticks.first(),   300ul,  "correct value for first");
    is(ticks.last(),    700ul,  "correct value for last");

    ticks.push(1); ticks.push(2); ticks.push(3); 
    ticks.push(4); ticks.push(5);
    ok(!ticks.empty(),          "ticks is now not empty");
    is(ticks.count(),   5,      "ticks.count remains at 5");
    show_ticks();
    is(ticks.first(),   1ul,    "correct value for first");
    is(ticks.last(),    5ul,    "correct value for last");

    ticks.clear();
    ok(ticks.empty(),           "ticks empty after clear");
    is(ticks.count(),   0,      "count 0 after clear");

    done_testing();
}
