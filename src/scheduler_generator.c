#include <msp430.h>
#include <stdint.h>

#define MAX_TASKS 8u
#define MAX_SLOTS 128u

typedef void (*task_fn_t)(void);

typedef struct
{
    const char *name;
    uint16_t period_ms;
    uint16_t slice_ms;
    uint16_t offset_ms;  // computed to avoid collisions
    task_fn_t func;
} task_def_t;

typedef struct
{
    task_fn_t func;
    uint32_t start_ms;
    uint16_t duration_ms;
} slot_t;

static task_def_t tasks[MAX_TASKS];
static slot_t schedule[MAX_SLOTS];
static uint8_t num_tasks = 0u;
static uint8_t num_slots = 0u;

static volatile uint32_t sys_ms = 0u;
static uint32_t hyperperiod_ms = 0u;

/* ---------- User tasks ---------- */
void task_1(void)
{
    P1OUT ^= BIT0;
}   // LED P1.0

void task_2(void)
{
    P1OUT ^= BIT1;
}   // LED P1.1

void task_3(void)
{
    P1OUT ^= BIT0;
}   // LED P1.0

/* ---------- Add task ---------- */
void add_task(const char *name, task_fn_t fn, uint16_t period, uint16_t slice)
{
    if (num_tasks >= MAX_TASKS)
        return;
    tasks[num_tasks++] = (task_def_t){ name, period, slice, 0, fn };
}

/* ---------- GCD / LCM ---------- */
static uint32_t gcd(uint32_t a, uint32_t b)
{
    while(b)
    {
        uint32_t t = b;
        b = a % b;
        a = t;
    }
    return a;
}

static uint32_t lcm(uint32_t a, uint32_t b)
{
    return a / gcd(a, b) * b;
}

uint32_t compute_hyperperiod(void)
{
    if (num_tasks == 0)
        return 0u;
    uint32_t hyper = tasks[0u].period_ms;
    for (uint8_t i = 1u; i < num_tasks; i++)
        hyper = lcm(hyper, tasks[i].period_ms);
    return hyper;
}

/* ---------- Compute offsets automatically ---------- */
void compute_offsets(void)
{
    uint16_t accumulated_slice = 0u;
    // sort tasks by period descending (simple bubble)
    for (uint8_t i = 0; i < num_tasks - 1; i++)
    {
        for (uint8_t j = i + 1; j < num_tasks; j++)
        {
            if (tasks[j].period_ms > tasks[i].period_ms)
            {
                task_def_t tmp = tasks[i];
                tasks[i] = tasks[j];
                tasks[j] = tmp;
            }
        }
    }
    // assign offsets
    for (uint8_t i = 0u; i < num_tasks; i++)
    {
        tasks[i].offset_ms = accumulated_slice % tasks[i].period_ms;
        accumulated_slice += tasks[i].slice_ms;
    }
}

/* ---------- Build schedule table ---------- */
void build_schedule(void)
{
    hyperperiod_ms = compute_hyperperiod();
    num_slots = 0;

    for (uint8_t i = 0; i < num_tasks; i++)
    {
        task_def_t *t = &tasks[i];
        uint32_t instances = hyperperiod_ms / t->period_ms;
        for (uint32_t n = 0; n < instances; n++)
        {
            uint32_t start = t->offset_ms + n * t->period_ms;
            if (num_slots < MAX_SLOTS)
            {
                schedule[num_slots++] = (slot_t){ t->func, start, t->slice_ms };
            }
        }
    }

    // sort slots by start_ms
    for (uint8_t i = 0; i < num_slots - 1; i++)
    {
        for (uint8_t j = i + 1; j < num_slots; j++)
        {
            if (schedule[j].start_ms < schedule[i].start_ms)
            {
                slot_t tmp = schedule[i];
                schedule[i] = schedule[j];
                schedule[j] = tmp;
            }
        }
    }
}

/* ---------- Timer ISR ---------- */
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER0_A0_VECTOR
__interrupt void timer_0_a0_isr (void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) timer_0_a0_isr (void)
#else
#error Compiler not supported!
#endif
{
    sys_ms++;
    __bic_SR_register_on_exit(LPM0_bits);
}

/* ---------- GPIO ---------- */
void gpio_init(void)
{
    PM5CTL0 &= ~LOCKLPM5;
    P1DIR |= BIT0 | BIT1;
    P1OUT &= ~(BIT0 | BIT1);
}

/* ---------- Timer ---------- */
void systick_init(void)
{
    TA0CCR0 = 999;
    TA0CCTL0 = CCIE;
    TA0CTL = TASSEL__SMCLK | MC__UP | TACLR;
}

/* ---------- Scheduler execution ---------- */
static uint8_t slot_idx = 0;

void run_scheduler(void)
{
    if (slot_idx >= num_slots)
        slot_idx = 0;
    slot_t *s = &schedule[slot_idx];
    if ((sys_ms % hyperperiod_ms) == s->start_ms)
    {
        uint32_t start_ms = sys_ms;
        s->func();
        if ((sys_ms - start_ms) >= s->duration_ms)
        {
            // exceeded slice
        }
        slot_idx++;
    }
}

/* ---------- Clock ---------- */
void clk_init(void)
{
    CSCTL0_H = CSKEY >> 8;
    CSCTL1 = DCOFSEL_0;
    CSCTL2 = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK;
    CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;
    CSCTL0_H = 0;
}

/* ---------- Main ---------- */
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;

    clk_init();
    gpio_init();
    systick_init();

    add_task("T1", task_1, 10, 2);
    add_task("T2", task_2, 50, 5);
    add_task("T3", task_3, 100, 10);

    compute_offsets();
    build_schedule();

    __enable_interrupt();

    while(1)
    {
        __disable_interrupt();
        __bis_SR_register(LPM0_bits | GIE);
        __enable_interrupt();
        run_scheduler();
    }
}
