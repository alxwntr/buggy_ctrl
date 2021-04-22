#include <Arduino.h>

#include "MotorArray.h"

/* Motor array:
    {Front left,
     Front right,
     Back left,
     Back right}

     Entries in the form [motor_index, RHS?, EncA, EncB, DirSel, Drive]

     Need to put correct pins in.

     The encoder pins need to match setup_motors below.
*/

MotorArray motors = {{
  { 0, false, A1, A2, 9, 10 },
  { 1, true, A3, A4, 11, 12 },
}};

//-------------------------
//  Interrupt functions
//-------------------------

//16cpr
static void ISR_motor0_pinA() { motors[0].get_encoder().handle_irq(PinAInterrupt); }
//static void ISR_motor0_pinB() { motors[0].get_encoder().handle_irq(PinBInterrupt); }
static void ISR_motor1_pinA() { motors[1].get_encoder().handle_irq(PinAInterrupt); }
//static void ISR_motor1_pinB() { motors[1].get_encoder().handle_irq(PinBInterrupt); }

static void
do_attach_isr (int pin, void (*isr)(void))
{
    attachInterrupt(digitalPinToInterrupt(pin), isr, CHANGE);
}

void
setup_pins()
{
    motors[0].setup_pins();
    motors[1].setup_pins();

    /* This need to match the encoder pins above */
    do_attach_isr(A1, ISR_motor0_pinA);
    //do_attach_isr(A2, ISR_motor0_pinB);
    do_attach_isr(A3, ISR_motor1_pinA);
    //do_attach_isr(A4, ISR_motor1_pinB);
}
