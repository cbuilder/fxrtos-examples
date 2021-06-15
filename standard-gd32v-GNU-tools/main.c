/**
  ******************************************************************************
  *  @file   main.c
  *  @brief  FX-RTOS demo application.
  *  The demo creates three LED-blinking threads, one GPT interrupt used
  *  to turn on all the leds every 5 seconds.
  *
  ******************************************************************************

    Copyright (c) 2020 Eremex Ltd.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
    3. The name of the author may not be used to endorse or promote products
       derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
    IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
    OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
    IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
    NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
    DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
    THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
    THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *****************************************************************************/

#include <gd32vf103.h>
#include <system_gd32vf103.h>
#include <gd32vf103_timer.h>
#include <n200_func.h>

#include "FXRTOS.h"

//
// System timer is clocked with the (HCLK/4)
//
#define MTIMER_FREQ (SystemCoreClock/4)
#define TICK_FREQ 1000U

#define LED_R_PIN 13
#define LED_G_PIN 1
#define LED_B_PIN 2

static fx_sem_t sem1;
static fx_sem_t sem2;
static fx_sem_t sem3;
static fx_sem_t sem4;

static volatile uint32_t* const gpioa_ctl0 = (uint32_t*)0x40010800;
static volatile uint32_t* const gpioa_out_ctl = (uint32_t*)0x4001080C;
static volatile uint32_t* const gpioc_ctl1 = (volatile uint32_t*)0x40011004;
static volatile uint32_t* const gpioc_out_ctl = (volatile uint32_t*)0x4001100C;
static volatile uint64_t* const mtimecmp = (volatile uint64_t*)0xd1000008;
static volatile uint64_t* const mtime = (volatile uint64_t*)0xd1000000;

//
// Need if using HAL_CPU_INTR = RV32I_V2 to generate software interrupt
//
void hal_pic_msip_set(unsigned int req)
{
    static volatile uint32_t* const msip = (volatile uint32_t*) 0xd1000ffc;
    *msip = req;
}

extern uintptr_t hw_cpu_mcause_get(void);
extern void hal_intr_handler(uint32_t mcause);
//
// Using with eclic equipped MCU to pass mcause parameter
// to hal_intr_handler without changing the last
//
void _hal_intr_handler_entry(void)
{
     hal_intr_handler(hw_cpu_mcause_get());
}

//
// Configure timer for the next tick
//
static inline void mtimer_setup(void)
{
    *mtimecmp = *mtime + MTIMER_FREQ/TICK_FREQ;
}

//
// CPU specicic function to disable mtimer interrupt
//
void hal_timer_pre_tick(void)
{
    eclic_disable_interrupt(CLIC_INT_TMR);
}

//
// CPU specicic function to configure next tick and enable mtimer interrupt
//
void hal_timer_post_tick(void)
{
    mtimer_setup();
    eclic_enable_interrupt(CLIC_INT_TMR);
}

static void green(int off_on)
{
    uint32_t v = *gpioa_ctl0;
    v &= ~(0xF << LED_G_PIN*4);
    v |= (0x3 << LED_G_PIN*4);
    *gpioa_ctl0 = v;
     
    //
    // Save state
    //
    v = *gpioa_out_ctl;
    //
    // clear bit
    //
    v &= ~(1 << LED_G_PIN);
    //
    // set value inverted
    //
    v |= (~off_on << LED_G_PIN);
    *gpioa_out_ctl = v;
}

static void blue(int off_on)
{
    uint32_t v = *gpioa_ctl0;
    v &= ~(0xF << LED_B_PIN*4);
    v |= (0x3 << LED_B_PIN*4);
    *gpioa_ctl0 = v;

    v = *gpioa_out_ctl;
    v &= ~(1 << LED_B_PIN);
    v |= (~off_on << LED_B_PIN);
    *gpioa_out_ctl = v;
}

static void red(int off_on)
{
    uint32_t v = *gpioc_ctl1;
    v &= ~(0xF << 20);
    v |= (0x3 << 20);
    *gpioc_ctl1 = v;

    v = *gpioc_out_ctl;
    v &= ~(1 << LED_R_PIN);
    v |= (~off_on << LED_R_PIN);
    *gpioc_out_ctl = v;
}

/*!
 * Simulate fade effect with pulse width modulation.
 * @param[in] ctrl LED's on/off control function
 * @param[in] duration Duration must be divisable by 10
 * @param[in] dir Direction must be either 1 for "fade in" or -1 for "fade out"
 */
static void fade(void(*ctrl)(int), const unsigned int duration, const signed int dir)
{
    const unsigned int T = 10;
    const unsigned int n = duration / T;
    unsigned int s = T * (unsigned int)(dir > 0);
    unsigned int m = 0;
    unsigned int i = 0;

    for (i = 0; i < n; ++i)
    {
        const unsigned int f = (i * T) / n;
        s += ((int)(m != f)) * -dir;
        m = f;

        ctrl(0);
        fx_thread_sleep(s);
        ctrl(1);
        fx_thread_sleep(T - s);  
    }
}

void g_thread_fn(void* arg)
{
    (void)arg;
    fx_sem_post(&sem1);
    while (1)
    {
        
        fx_sem_wait(&sem1, NULL);
        fade(green, 600, 1);
        fade(green, 600, -1);
        green(0);
        fx_sem_post(&sem2);
        
    }
}

void b_thread_fn(void* arg)
{
    (void)arg;
    
    while (1)
    {
        fx_sem_wait(&sem2, NULL);
        fade(blue, 600, 1);
        fade(blue, 600, -1);
        blue(0);
        fx_sem_post(&sem3);
    }
}

void r_thread_fn(void* arg)
{
    (void)arg;
    while (1)
    { 
        fx_sem_wait(&sem3, NULL);
        fade(red, 600, 1);
        fade(red, 600, -1);
        red(0);
        fx_sem_post(&sem1);
    }
}

void irq_thread_fn(void* arg)
{
    (void)arg;

    //
    // TIMER5 Initialization
    //
    TIMER_INTF(TIMER5) = 0;
    TIMER_CTL0(TIMER5) = 0;
    TIMER_PSC(TIMER5) = 10800;
    TIMER_CAR(TIMER5) = TIMER_CNT(TIMER5) + 50000;
    TIMER_DMAINTEN(TIMER5) = TIMER_DMAINTEN_UPIE;
    TIMER_CTL0(TIMER5) = TIMER_CTL0_CEN;
    //
    // Enable TIMER5_IRQn
    //
    eclic_set_intattr(TIMER5_IRQn, ECLIC_INT_ATTR_TRIG_EDGE);
    eclic_clear_pending(TIMER5_IRQn);
    eclic_enable_interrupt(TIMER5_IRQn);

    while (1)
    {
        fx_sem_wait(&sem4, NULL);
        green(1);
        red(1);
        blue(1);
    }
}

//
// Application definition. Called by the kernel after initialization
//
void fx_app_init(void)
{
    static fx_thread_t t1;
    static fx_thread_t t2;
    static fx_thread_t t3;
    static fx_thread_t t4;
    static int t1_stk[0x200];
    static int t2_stk[0x200];
    static int t3_stk[0x200];
    static int t4_stk[0x200];

    green(0);
    blue(0);
    red(0);

    mtimer_setup();
    eclic_set_intattr(CLIC_INT_TMR, 0);
    eclic_enable_interrupt(CLIC_INT_TMR);

    fx_sem_init(&sem1, 0, 1, FX_SYNC_POLICY_FIFO);
    fx_sem_init(&sem2, 0, 1, FX_SYNC_POLICY_FIFO);
    fx_sem_init(&sem3, 0, 1, FX_SYNC_POLICY_FIFO);
    fx_sem_init(&sem4, 0, 1, FX_SYNC_POLICY_FIFO);
    fx_thread_init(&t1, g_thread_fn, NULL, 5, t1_stk, sizeof(t1_stk), false);
    fx_thread_init(&t2, b_thread_fn, NULL, 5, t2_stk, sizeof(t2_stk), false);
    fx_thread_init(&t3, r_thread_fn, NULL, 5, t3_stk, sizeof(t3_stk), false);
    fx_thread_init(&t4, irq_thread_fn, NULL, 1, t4_stk, sizeof(t4_stk), false);
}

//
// Program entry point. Interrupts are disabled by startup code
//
int main(void)
{
    SystemInit();

    //
    // GPIOA, GPIOC clock enable
    //
    RCU_APB2EN |= RCU_APB2EN_PCEN | RCU_APB2EN_PAEN;
    //
    // TIMER5 clock enable
    //
    RCU_APB1EN |= RCU_APB1EN_TIMER5EN;

    eclic_init(ECLIC_NUM_INTERRUPTS);
    eclic_mode_enable();
   
    fx_kernel_entry();
    return 0;
}

//
// None interrupts except timer are allowed
//
void fx_intr_handler(void)
{
    switch(hal_intr_get_current_vect())
    {
    case TIMER5_IRQn:    
        TIMER_INTF(TIMER5) = 0;
        //
        // Period 5 seconds
        //
        TIMER_CAR(TIMER5) = TIMER_CNT(TIMER5) + 50000;
        fx_sem_post(&sem4);
        break;
    default:
        break;
    }
}

//
// We do not handle exceptions
//
void hal_trap_handler(void)
{
    for (;;) ;
}
