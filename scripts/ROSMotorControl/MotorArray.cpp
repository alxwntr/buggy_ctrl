#include <Arduino.h>

#include "MotorArray.h"

/* Motor array:
    {Front left,
     Front right,
     Back left,
     Back right}

     Entries in the form [RHS?, EncA, EncB, M1, M2]

     There are two levels of braces on the outside because std::array is
     a structure (class) containing an array, not a bare array.
*/

/* XXX I have guessed at suitable pins for encB. I believe any spare
 * pins should work (except 5). */
MotorArray motors = {{
  { false, A0, 3, 12, 11 },
  { true, A1, 6, 10, 9 },
  { false, A2, 7, 2, 4 },
  { true, A3, 8, A4, A5 },
}}; 

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
