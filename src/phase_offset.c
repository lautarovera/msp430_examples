/*
 * MSP430FR5994 Deterministic Phase-Offset Scheduler
 * -------------------------------------------------
 * - TimerA0 generates 1 ms system tick (SMCLK = 1 MHz)
 * - Cooperative (non-preemptive) superloop
 * - Each task has: period_ms, slice_ms, phase_offset_ms
 * - Phase offsets chosen to avoid overlap → zero jitter schedule
 *
 * Tasks:
 *   T1: Blink LED1 every 10ms (slice 1ms, offset 0ms)
 *   T2: Blink LED1 every 100ms (slice 5ms, offset 2ms)
 *   T3: Blink LED2 every 500ms (slice 20ms, offset 10ms)
 */

#include <msp430.h>
#include <stdint.h>

/* ---------- Configuration ---------- */
#define TICK_MS 1
#define MAX_TASKS 8

typedef void (*task_fn_t)(uint32_t now_ms);

typedef struct {
    task_fn_t fn;
    uint32_t  period_ms;
    uint32_t  slice_ms;
    uint32_t  phase_offset_ms;
    uint32_t  next_run_ms;
} task_t;

/* ---------- User tasks ---------- */
static void task_fast(uint32_t now_ms);
static void task_medium(uint32_t now_ms);
static void task_slow(uint32_t now_ms);

/* ---------- Scheduler state ---------- */
static volatile uint32_t sys_ms = 0;
static task_t tasks[MAX_TASKS];
static uint8_t task_count = 0;

/* ---------- Clock / GPIO / Timer ---------- */
void CLK_Init(void)
{
    CSCTL0_H = CSKEY >> 8;            // unlock
    CSCTL1 = DCOFSEL_0;               // DCO = 1 MHz
    CSCTL2 = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK;
    CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;
    CSCTL0_H = 0;                     // lock
}

void GPIO_Init(void)
{
    PM5CTL0 &= ~LOCKLPM5;             // enable GPIO
    P1DIR |= BIT0 | BIT1;             // P1.0 and P1.1 outputs
    P1OUT &= ~(BIT0 | BIT1);
}

void TimerA0_Init_1ms(void)
{
    TA0CCR0 = 999;                    // 1 MHz / 1000 = 1 kHz => 1 ms
    TA0CCTL0 = CCIE;                  // CCR0 interrupt enable
    TA0CTL = TASSEL__SMCLK | MC__UP | TACLR;
}

/* ---------- Task registration ---------- */
int scheduler_register_task(task_fn_t fn, uint32_t period_ms, uint32_t slice_ms, uint32_t phase_offset_ms)
{
    if (task_count >= MAX_TASKS) return -1;
    tasks[task_count].fn = fn;
    tasks[task_count].period_ms = period_ms;
    tasks[task_count].slice_ms = slice_ms;
    tasks[task_count].phase_offset_ms = phase_offset_ms;
    tasks[task_count].next_run_ms = phase_offset_ms;
    task_count++;
    return 0;
}

/* ---------- Timer ISR: increments system tick ---------- */
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer0_A0_ISR (void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) Timer0_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    sys_ms++;
    __bic_SR_register_on_exit(LPM0_bits);
}

/* ---------- Main superloop ---------- */
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;

    CLK_Init();
    GPIO_Init();
    TimerA0_Init_1ms();

    /* Register tasks with deterministic offsets */
    scheduler_register_task(task_fast,   10,  1,  0);   // every 10 ms, 1 ms slice, offset 0
    scheduler_register_task(task_medium, 100, 5,  2);   // every 100 ms, 5 ms slice, offset 2
    scheduler_register_task(task_slow,   500, 20, 10);  // every 500 ms, 20 ms slice, offset 10

    __enable_interrupt();

    while (1)
    {
        uint8_t have_work = 0;
        uint32_t now_ms;

        /* Atomically read current time */
        __disable_interrupt();
        now_ms = sys_ms;
        __enable_interrupt();

        for (uint8_t i = 0; i < task_count; i++)
        {
            if ((int32_t)(now_ms - tasks[i].next_run_ms) >= 0)
            {
                /* Run this task */
                tasks[i].fn(now_ms);

                /* Schedule next activation */
                tasks[i].next_run_ms += tasks[i].period_ms;

                have_work = 1;
            }
        }

        if (!have_work)
        {
            /* Sleep until next interrupt */
            __bis_SR_register(LPM0_bits | GIE);
        }
    }
}

/* ---------- User task implementations ---------- */
static void task_fast(uint32_t now_ms)
{
    /* Simulate small work (LED toggle) */
    P1OUT ^= BIT0;
    (void)now_ms;
}

static void task_medium(uint32_t now_ms)
{
    P1OUT ^= BIT0;  // toggle LED1 slower
    (void)now_ms;
}

static void task_slow(uint32_t now_ms)
{
    P1OUT ^= BIT1;  // toggle LED2
    (void)now_ms;
}
