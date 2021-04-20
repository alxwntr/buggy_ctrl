#include <Arduino.h>

#include "MotorArray.h"

/* Motor array:
    {Front left,
     Front right,
     Back left,
     Back right}

     Entries in the form [motor_index, RHS?, EncA, EncB, DirSel, Drive]

     Need to put correct pins in.
*/

MotorArray motors = {{
  { 0, false, A1, A2, 9, 10 },
  { 1, true, A3, A4, 11, 12 },
}};

//-------------------------
//  Interrupt functions
//-------------------------

static void ISR1() { motors[0].get_encoder().handle_irq(PinAInterrupt); }
static void ISR2() { motors[0].get_encoder().handle_irq(PinBInterrupt); }
static void ISR3() { motors[1].get_encoder().handle_irq(PinAInterrupt); }
static void ISR4() { motors[1].get_encoder().handle_irq(PinBInterrupt); }

void
setup_motors()
{
    motors[0].setup_pins(ISR1);
    motors[1].setup_pins(ISR3);
}
