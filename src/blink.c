#include <msp430.h>
#include <stdbool.h>

/* Half a second at 1MHz */
#define BLINK_CYCLES 500000L

int main(void) {
    /* Stop the watchdog timer */
    WDTCTL = WDTPW | WDTHOLD;
    /* Allow changes to port registers */
    PM5CTL0 &= ~LOCKLPM5;

    /* Set pin 1.0 as output */
    P1DIR |= BIT0;
    /* Clear pin 1.0 (make it low)*/
    P1OUT &= ~BIT0;

    while (true) {
        __delay_cycles(BLINK_CYCLES);
        /* Toggle pin 1.0 */
        P1OUT ^= BIT0;
    }
}