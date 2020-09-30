#include "tap.h"
#include "mock/Arduino.h"

using namespace tap::exports;

void mock_isr(void) { return; }

int
main(int argc, char **argv)
{
    clear_mock_results();
    pinMode(2, INPUT);

    mock_results_are({
        { {"pinMode", {{2, 0}}} },
    }, "testing pinMode");

    clear_mock_results();
    attachInterrupt(6, mock_isr);

    mock_results_are({
        { {"attachInterrupt", {{6, (intptr_t)mock_isr}}} },
    }, "testing attachInterrupt");

    done_testing();
}
