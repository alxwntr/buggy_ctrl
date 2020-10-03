#include <Arduino.h>

#include "MotorArray.h"

/* Motor array:
    {Front left,
     Front right,
     Back left,
     Back right}

     Entries in the form [RHS?, Enc, M1, M2]

     There are two levels of braces on the outside because std::array is
     a structure (class) containing an array, not a bare array.
*/

MotorArray motors = {{
  { false, A0, 12, 11 },
  { true, A1, 10, 9 },
  { false, A2, 2, 4 },
  { true, A3, A4, A5 },
}}; //Pins verified as working motor by motor, need to test all four together

//-------------------------
//  Interrupt functions
//-------------------------

static void ISR1() { motors[0].handle_irq(); }
static void ISR2() { motors[1].handle_irq(); }
static void ISR3() { motors[2].handle_irq(); }
static void ISR4() { motors[3].handle_irq(); }

void
setup_motors()
{
    motors[0].setup_pins(ISR1);
    motors[1].setup_pins(ISR2);
    motors[2].setup_pins(ISR3);
    motors[3].setup_pins(ISR4);
}
