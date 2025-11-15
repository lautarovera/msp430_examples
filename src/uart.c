#include <msp430.h>
#include <stdio.h>

int uart_putchar(int c) {
    while (!(UCA0IFG & UCTXIFG));  // Wait until buffer is ready
    UCA0TXBUF = c;                 // Transmit character
    return c;
}

int _write(int file, char *ptr, int len) {
    int i;
    for (i = 0; i < len; i++) {
        uart_putchar((int)ptr[i]);
    }
    return len;
}

void CLK_Init(void)
{
    // Startup clock system with max DCO setting ~8MHz
    CSCTL0_H = CSKEY_H;                     // Unlock CS registers
    CSCTL1 = DCOFSEL_6;           // Set DCO to 8MHz
    CSCTL2 = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK;
    CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;   // Set all dividers
    CSCTL0_H = 0;                           // Lock CS registers

    __delay_cycles(10000);  // Wait for clock set
}

void UART_init(void)
{
    // Configure GPIO
    P2SEL1 |= BIT0 + BIT1;              //Activate Pin for UART use
    P2SEL0 &= ~BIT0 + ~BIT1;            //Activate Pin for UART use

    // Configure USCI_A0 for UART mode
    UCA0CTLW0 = UCSWRST;                    // Put eUSCI in reset
    UCA0CTLW0 |= UCSSEL__SMCLK;             // CLK = SMCLK
    // Baud Rate calculation for 115200 bps @ 1 MHz SMCLK:
    // N = 1,000,000 / 115,200 ≈ 8.6805
    // UCBRx = INT(N / 16) = INT(0.5425) = 0
    // UCBRFx = INT(((N / 16) - UCBRx) * 16) = INT(0.5425 * 16) = 8 (0.68 error)
    // UCBRSx (Modulation) from Table 21-5 (best fit for 8.6805) = 0x20
    
    UCA0BRW = 0;                        // UCBRx = 0
    // UCOS16 (oversampling) | UCBRFx (8) | UCBRSx (0x20)
    UCA0MCTLW = UCOS16 | UCBRF_8 | 0x20;

    UCA0CTLW0 &= ~UCSWRST;                  // Initialize eUSCI
}

void app_uart(void)
{
    P3DIR |= BIT4;   // Set P3.4 as output
    P3SEL1 |= BIT4;  // Select SMCLK function
    P3SEL0 |= BIT4;

    CLK_Init();
    UART_init();

    while(1)
    {
        printf("Hello, MSP430 UART!\n\r");
    }
}

int main( void )
{
    WDTCTL = WDTPW | WDTHOLD;               // Stop watchdog timer
    PM5CTL0 &= ~LOCKLPM5;                   // Disable the GPIO power-on default high-impedance mode

    app_uart();
}
