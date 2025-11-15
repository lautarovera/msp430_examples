/*
 * Cooperative periodic scheduler with time-slice self-checks
 * MSP430FR5994 @ 1 MHz SMCLK
 *
 * - TA0 -> 1 ms tick interrupt
 * - Each task has: period_ms, slice_ms, and a pending counter
 * - Main loop runs flagged tasks cooperatively
 * - Tasks receive timestamp (now_ms) and self-check runtime
 */

#include <msp430.h>
#include <stdint.h>
#include <stdio.h>

#define MAX_TASKS   8
#define TICK_MS     1

typedef void (*task_fn_t)(uint32_t now_ms);

typedef struct {
    task_fn_t  fn;
    uint32_t   period_ms;
    uint32_t   slice_ms;     // allowed execution window
    volatile uint16_t pending;
} task_t;

static task_t tasks[MAX_TASKS];
static uint8_t task_count = 0;
static volatile uint32_t ms_ticks = 0;

/* -------- Utility macros -------- */
#define TIME_EXPIRED(start, limit) ((int32_t)((ms_ticks) - (start)) >= (int32_t)(limit))

/* -------- Prototypes -------- */
static void task_10ms(uint32_t now);
static void task_100ms(uint32_t now);
static void task_500ms(uint32_t now);

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

/* -------- Clock / GPIO / Timer -------- */
void CLK_Init(void)
{
    CSCTL0_H = CSKEY >> 8;
    CSCTL1 = DCOFSEL_0;  // DCO = 1 MHz
    CSCTL2 = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK;
    CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;
    CSCTL0_H = 0;
}

void GPIO_Init(void)
{
    PM5CTL0 &= ~LOCKLPM5;
    P1DIR |= BIT0 | BIT1;
    P1OUT &= ~(BIT0 | BIT1);
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

/* 1ms tick using TA0 */
void TimerA0_Init_1ms(void)
{
    TA0CCR0 = 999;
    TA0CCTL0 = CCIE;
    TA0CTL = TASSEL__SMCLK | MC__UP | TACLR;
}

/* -------- Scheduler API -------- */
int scheduler_register_task(task_fn_t fn, uint32_t period_ms, uint32_t slice_ms)
{
    if (!fn || period_ms == 0 || task_count >= MAX_TASKS) return -1;
    tasks[task_count].fn = fn;
    tasks[task_count].period_ms = period_ms;
    tasks[task_count].slice_ms = slice_ms;
    tasks[task_count].pending = 0;
    task_count++;
    return 0;
}

/* -------- Timer ISR -------- */
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer0_A0_ISR (void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) Timer0_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    static uint16_t local_ms[MAX_TASKS];
    uint8_t i;

    ms_ticks++;

    for (i = 0; i < task_count; i++) {
        local_ms[i]++;
        if (local_ms[i] >= (uint16_t)tasks[i].period_ms) {
            local_ms[i] = 0;
            if (tasks[i].pending < 0xFFFF)
                tasks[i].pending++;
        }
    }

    __bic_SR_register_on_exit(LPM0_bits);
}

/* -------- Superloop -------- */
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;

    CLK_Init();
    GPIO_Init();
    UART_init();

    scheduler_register_task(task_10ms,  10,  2);
    scheduler_register_task(task_100ms, 100, 10);
    scheduler_register_task(task_500ms, 500, 50);

    TimerA0_Init_1ms();
    __enable_interrupt();

    while (1)
    {
        uint8_t i;
        uint8_t have_work = 0;

        __disable_interrupt();
        for (i = 0; i < task_count; i++) {
            if (tasks[i].pending) { have_work = 1; break; }
        }
        if (!have_work)
            __bis_SR_register(LPM0_bits | GIE);
        __enable_interrupt();

        for (i = 0; i < task_count; i++) {
            uint16_t run_cnt = 0;

            __disable_interrupt();
            if (tasks[i].pending) {
                run_cnt = tasks[i].pending;
                tasks[i].pending = 0;
            }
            __enable_interrupt();

            while (run_cnt--) {
                uint32_t now = ms_ticks;
                tasks[i].fn(now);
            }
        }
    }
}

/* -------- Example tasks with self-timing -------- */

static void task_10ms(uint32_t now)
{
    uint32_t start = now;

    while (!TIME_EXPIRED(start, 2))  // slice_ms = 2 ms
    {
        // Simulate work
        printf("[%d]T_10ms\n\r", now);
    }
}

static void task_100ms(uint32_t now)
{
    uint32_t start = now;

    while (!TIME_EXPIRED(start, 10))  // slice_ms = 10 ms
    {
        // Simulate work
        P1OUT ^= BIT0;
        printf("[%d]T_100ms\n\r", now);
    }
}

static void task_500ms(uint32_t now)
{
    uint32_t start = now;

    while (!TIME_EXPIRED(start, 50))  // slice_ms = 50 ms
    {
        // Simulate work
        P1OUT ^= BIT1;
        printf("[%d]T_500ms\n\r", now);
    }
}
