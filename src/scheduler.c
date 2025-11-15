/*
 * Cooperative periodic task scheduler for MSP430FR5994
 * - SMCLK = 1 MHz (DCO)
 * - TA0 CCR0 => 1 ms tick
 * - ISR increments per-task pending counters (volatile)
 * - Main loop polls counters and calls task functions (cooperative)
 *
 * Key patterns:
 * - Keep ISR minimal and use small static counters inside ISR
 * - Task counters are atomic-ish: accessed in main with interrupts briefly disabled
 * - ISR calls __bic_SR_register_on_exit(LPM0_bits) to wake main loop
 */

#include <msp430.h>
#include <stdint.h>
#include <stddef.h>

#define MAX_TASKS    8   // increase if needed
#define TICK_MS      1   // system tick in ms

/* Task type: function pointer with void(void) signature */
typedef void (*task_fn_t)(void);

typedef struct {
    task_fn_t  fn;         // function to run
    uint32_t   period_ms;  // period in ms (must be multiple of TICK_MS)
    volatile uint16_t pending; // pending executions queued by ISR (incremented in ISR)
} task_t;

/* ---------- User task prototypes (examples) ---------- */
static void task_10ms(void);
static void task_100ms(void);
static void task_500ms(void);

/* ---------- Scheduler storage ---------- */
static task_t tasks[MAX_TASKS];
static uint8_t  task_count = 0;

/* Register a periodic task. Returns 0 on success, -1 on failure. */
int scheduler_register_task(task_fn_t fn, uint32_t period_ms)
{
    if (!fn || period_ms == 0 || task_count >= MAX_TASKS) return -1;
    tasks[task_count].fn = fn;
    tasks[task_count].period_ms = period_ms;
    tasks[task_count].pending = 0;
    task_count++;
    return 0;
}

/* ---------- Clock / GPIO / Timer init ---------- */
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
    PM5CTL0 &= ~LOCKLPM5;             // enable GPIO (FRAM devices)
    P1DIR |= BIT0 | BIT1;             // P1.0 and P1.1 outputs
    P1OUT &= ~(BIT0 | BIT1);
}

/* Setup TA0 CCR0 to create 1 ms tick:
 * SMCLK = 1 MHz, use no divider -> CCR0 = 1000-1 = 999 for 1 ms
 * However we prefer smaller CCR0 to keep timer resolution low-cost on 16-bit: use TA0CCR0 = 999
 */
void TimerA0_Init_1ms(void)
{
    TA0CCR0 = 999;                    // 1 MHz / 1000 = 1 kHz => 1 ms
    TA0CCTL0 = CCIE;                  // CCR0 interrupt enable
    TA0CTL = TASSEL__SMCLK | MC__UP | TACLR; // SMCLK, up mode, clear TAR
}

/* ---------- ISR: keep very small ----------
 * - static local counters to avoid frequent FRAM writes
 * - increment per-task pending counters when their period elapses
 * - clear LPM0 bits on exit so main loop runs
 *
 * Implementation detail: maintain local millisecond accumulator and for each registered
 * task fire when multiple-of-period is reached. Because tasks[] is in RAM and task_count
 * is small, we compute per-task triggers by incrementing an internal counter array.
 */
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer0_A0_ISR (void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) Timer0_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    /* Local static ms accumulator per registered task (keeps CPU/RAM access inside ISR fast) */
    static uint16_t local_ms[MAX_TASKS];
    uint8_t i;

    /* Increment local counters and set pending when period reached */
    for (i = 0; i < task_count; i++)
    {
        local_ms[i]++;  // increments in fast SRAM
        if (local_ms[i] >= (uint16_t)tasks[i].period_ms) {
            local_ms[i] = 0;
            /* increment pending counter (volatile) -- small variable in RAM */
            if (tasks[i].pending < 0xFFFF) tasks[i].pending++;
        }
    }

    /* Wake up main loop after ISR */
    __bic_SR_register_on_exit(LPM0_bits);
}

/* ---------- Main superloop ----------
 * - Registers tasks
 * - In loop, disables interrupts briefly to snapshot and decrement pending counters safely
 * - Calls task functions outside the disabled-interrupt section (cooperative)
 * - Enters LPM0 when no pending work (but checks to avoid race condition)
 */
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;     // stop watchdog

    CLK_Init();
    GPIO_Init();

    /* Register tasks (periods in ms). Period must be >= TICK_MS and integer ms. */
    scheduler_register_task(task_10ms,  10);
    scheduler_register_task(task_100ms, 100);
    scheduler_register_task(task_500ms, 500);

    TimerA0_Init_1ms();

    __enable_interrupt();

    while (1)
    {
        uint8_t i;
        uint8_t have_work = 0;

        /* Check if any pending tasks exist atomically. If none, enter LPM0.
         * We disable interrupts briefly to avoid a race where ISR sets pending
         * between checking and sleeping.
         */
        __disable_interrupt();
        for (i = 0; i < task_count; i++) {
            if (tasks[i].pending) { have_work = 1; break; }
        }
        if (!have_work) {
            /* sleep until next tick (ISR will wake via __bic_SR_register_on_exit) */
            __bis_SR_register(LPM0_bits | GIE);
        }
        __enable_interrupt();

        /* Execute tasks that have pending counts.
         * We snapshot and decrement pending in a short atomic window, then call handlers
         * while interrupts are enabled so ISR keeps running.
         */
        for (i = 0; i < task_count; i++) {
            uint16_t run_cnt = 0;

            /* snapshot/decrement atomically */
            __disable_interrupt();
            if (tasks[i].pending) {
                run_cnt = tasks[i].pending;
                tasks[i].pending = 0;   // consume all pending occurrences (coalesced execution)
            }
            __enable_interrupt();

            /* run the task 'run_cnt' times (usually 0 or 1). Keep each invocation short. */
            while (run_cnt--) {
                tasks[i].fn();
            }
        }

        /* optional small nop or background work */
        __no_operation();
    }
}

/* ---------- Example user tasks ----------
 * Keep these short (non-blocking). If a task is long, it will delay other tasks.
 * If you need to preserve missed executions, use a different policy than "coalesce" above.
 */
static void task_10ms(void)
{
    /* Do nothing */
    __no_operation();
}

static void task_100ms(void)
{
    /* Toggle P1.0 */
    P1OUT ^= BIT0;
}

static void task_500ms(void)
{
    /* Toggle P1.1 */
    P1OUT ^= BIT1;
}

/* ---------- Notes & limitations ----------
 * 1) Tasks are cooperative: they must return quickly. If a task blocks for longer than
 *    the smallest scheduling period, other tasks will be delayed or missed.
 *
 * 2) ISR is minimal: local static counters + increment task[i].pending (volatile).
 *    Avoid FRAM writes in ISR; keep ISR code small and data in SRAM.
 *
 * 3) Pending counters are coalesced: if ISR increments pending multiple times before
 *    main consumes it, we call the task multiple times (run_cnt times). If you prefer
 *    "only run once" semantics, change the consumption to set pending = 0 and run once.
 *
 * 4) Race safety: we briefly disable interrupts when snapshotting/decrementing pending
 *    counters. This window is very short. If you need lock-free atomic ops, adapt to
 *    the platform word-size and use 16-bit atomic access patterns.
 *
 * 5) Timing accuracy: using DCO at 1 MHz is OK for many apps. For strict timekeeping,
 *    use an external crystal for SMCLK/ACLK or periodically calibrate DCO.
 *
 * 6) Memory/stack: keep stack usage minimal in tasks and avoid calling heavy library
 *    functions inside tasks (printf, floating point, etc.).
 *
 * 7) Scaling: MAX_TASKS limits the number of tasks; task table is static to avoid dynamic alloc.
 *
 * 8) If you need priorities, you can iterate tasks in priority order or add a priority field.
 *
 * 9) Low-power: LPM0 is used; TA0 (SMCLK) runs in LPM0. If you move to deeper LPMs,
 *    ensure the timer source remains active.
 *
 * This pattern is lightweight, deterministic, and works well for embedded systems that
 * do periodic/cooperative-style processing without preemption.
 */
