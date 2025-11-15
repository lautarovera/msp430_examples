#include <msp430.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

volatile bool flag_100ms = false;
volatile bool flag_500ms = false;

typedef enum {
    CLK_1MHZ,
    CLK_8MHZ
} ClockSpeed_t;

ClockSpeed_t systemClock = CLK_1MHZ;

static void SetClkTo8MHz(void)
{
    // Startup clock system with max DCO setting ~8MHz
    CSCTL0_H = CSKEY_H;                     // Unlock CS registers
    CSCTL1 = DCOFSEL_6;                     // Set DCO to 8MHz
    CSCTL2 = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK;
    CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;   // Set all dividers
    CSCTL0_H = 0;                           // Lock CS registers

    __delay_cycles(10000);  // Wait for clock set
}

static void SetClkTo1MHz(void)
{
    // Configure DCO = 1 MHz
    CSCTL0_H = CSKEY >> 8;                       // Unlock CS registers
    CSCTL1 = DCOFSEL_0;                          // DCO = 1 MHz
    CSCTL2 = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK;
    CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;        // No dividers
    CSCTL0_H = 0;                                // Lock CS registers
}

void Clk_Init(ClockSpeed_t speed)
{
    systemClock = speed;

    switch (speed)
    {
        case CLK_1MHZ:
            SetClkTo1MHz();
            break;
        case CLK_8MHZ:
            SetClkTo8MHz();
            break;
        default:
            SetClkTo1MHz();
            break;
    }
}

void Delay_ms(uint16_t ms)
{
    while (ms--)
    {
        switch (systemClock)
        {
            case CLK_1MHZ:
                __delay_cycles(1000);
                break;
            case CLK_8MHZ:
                __delay_cycles(8000);
                break;
            default:
                __delay_cycles(1000);
                break;
        }
    }
}

void SystemTick_Init(void)
{
    // Configure Timer_A0 for 1ms system tick
    TA0CCTL0 = CCIE;
    TA0CCR0  = 124;                        // 1 ms @ 125 kHz
    TA0CTL   = TASSEL__SMCLK | ID__8 | MC__UP | TACLR;
}

void GPIO_Init(void)
{
    P1DIR |= BIT0 | BIT1;    // P1.0 and P1.1 as outputs
    P1OUT &= ~(BIT0 | BIT1); // Initialize LEDs off
}

int main( void )
{
    WDTCTL = WDTPW | WDTHOLD;               // Stop watchdog timer
    PM5CTL0 &= ~LOCKLPM5;                   // Disable the GPIO power-on default high-impedance mode

    GPIO_Init();

    Clk_Init(CLK_1MHZ);

    Delay_ms(10);  // Wait for clock set

    SystemTick_Init();

    while(true)
    {
        // Enter LPM0 until an interrupt wakes CPU
        __disable_interrupt();
        if (!flag_100ms && !flag_500ms)
        {
            __bis_SR_register(LPM0_bits | GIE);  // Enter LPM0 with interrupts enabled
        }
        __enable_interrupt();

        // 100 ms task
        if (flag_100ms)
        {
            flag_100ms = false;
            P1OUT ^= BIT0;     // Toggle LED0
            // Add other 100 ms logic here
        }

        // 500 ms task
        if (flag_500ms)
        {
            flag_500ms = false;
            P1OUT ^= BIT1;     // Toggle LED1
            // Add other 500 ms logic here
        }
    }
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
    static uint16_t c100 = 0u, c500 = 0u;

    if (++c100 >= 100u) { c100 = 0u; flag_100ms = true; }
    if (++c500 >= 500u) { c500 = 0u; flag_500ms = true; }

    __bic_SR_register_on_exit(LPM0_bits);  // Exit LPM0
}
