#include <msp430.h>
#include <stdio.h>

void CLK_Init(void)
{
    // Startup clock system with max DCO setting ~8MHz
    CSCTL0_H = CSKEY_H;                     // Unlock CS registers
    CSCTL1 = DCOFSEL_6;                     // Set DCO to 8MHz
    CSCTL2 = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK;
    CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;   // Set all dividers
    CSCTL0_H = 0;                           // Lock CS registers

    __delay_cycles(10000);  // Wait for clock set
}

void app_timer(void)
{
    // P1.0 is output low by default
    P1OUT &= ~BIT0;
    P1DIR |= BIT0;

    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;

    TA0CCTL0 = CCIE;                        // TACCR0 interrupt enabled
    TA0CCR0 = 2000;
    TA0CTL = TASSEL__ACLK | MC__UP;         // ACKL, UP mode

    __bis_SR_register(GIE);                 // Enter LPM0 w/ interrupt
    __no_operation();                       // For debugger
}

int main( void )
{
    WDTCTL = WDTPW | WDTHOLD;               // Stop watchdog timer
    PM5CTL0 &= ~LOCKLPM5;                   // Disable the GPIO power-on default high-impedance mode

    CLK_Init();

    app_timer();
}

// Interrupt Service Routines

// Timer0_A0 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer0_A0_ISR (void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) Timer0_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    P1OUT ^= BIT0;  // toggle P1.0
}
