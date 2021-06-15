/**
  ******************************************************************************
  *  @file   entry.S
  *  @brief  Trap, interrupt entry functions.
  *
  ******************************************************************************
    Copyright (c) 2020 GigaDevice Semiconductor Inc.
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

#ifndef ENTRY_S
#define ENTRY_S

#include "riscv_encoding.h"

#define HAL_INTR_STACK_SIZE 0x400
#define CONTEXT_SIZE (29 * 4)

#define MOVE____GPRS(cmd)  \
    cmd  ra,  4(sp);   \
    cmd  x5,  8(sp);   \
    cmd  x6,  12(sp);  \
    cmd  x7,  16(sp);  \
    cmd  x28, 20(sp);  \
    cmd  x29, 24(sp);  \
    cmd  x30, 28(sp);  \
    cmd  x31, 32(sp);  \
    cmd  x10, 36(sp);  \
    cmd  x11, 40(sp);  \
    cmd  x12, 44(sp);  \
    cmd  x13, 48(sp);  \
    cmd  x14, 52(sp);  \
    cmd  x15, 56(sp);  \
    cmd  x16, 60(sp);  \
    cmd  x17, 64(sp);  \
    cmd  x8,  68(sp);  \
    cmd  x9,  72(sp);  \
    cmd  x18, 76(sp);  \
    cmd  x19, 80(sp);  \
    cmd  x20, 84(sp);  \
    cmd  x21, 88(sp);  \
    cmd  x22, 92(sp);  \
    cmd  x23, 96(sp);  \
    cmd  x24, 100(sp); \
    cmd  x25, 104(sp); \
    cmd  x26, 108(sp); \
    cmd  x27, 112(sp);  

#Save caller registers
.macro SAVE_CONTEXT
    addi    sp, sp, -CONTEXT_SIZE
    MOVE____GPRS(sw)
.endm

.macro RESTORE_CONTEXT
    MOVE____GPRS(lw)
    addi    sp, sp, CONTEXT_SIZE
.endm

.macro SAVE_EPC
    csrr    t0, mepc
    sw      t0, 0(sp)
.endm

.macro RESTORE_EPC
    lw      t0, 0(sp)
    csrw    mepc, t0 
.endm

//
// Trap entry point
//
    .section .text.trap
    .align 6
    .global trap_entry
trap_entry:
    SAVE_CONTEXT
    SAVE_EPC

    csrr    a0, mcause
    mv      a1, sp
    call    hal_trap_handler

    RESTORE_EPC
    RESTORE_CONTEXT
    mret

//
// IRQ entry point
//
    .section .text.irq	
    .align 2
    .global irq_entry
irq_entry:
    SAVE_CONTEXT
    /* Store mepc if it would be a nested interrupt */
    SAVE_EPC
    /* check and incement nesting counter */
    la      t0, g_hal_intr_nesting
    lw      t1, 0(t0)
    addi    t2, t1, 1
    sw      t2, 0(t0)
    /* if nesting counter was !=0, continue to serve interrupts in hal_intr_handler */
    bnez    t1, asynchronous_intr
    /* if nesting counter was == 0, save thread's sp to the g_hal_intr_stack_frame */
    la      t0, g_hal_intr_stack_frame
    sw      sp, 0(t0)
    /* load interrupt stack frame address to SP */
    la      sp, g_hal_intr_stack
    /* load the top of the interrupt stack to SP */
    addi    sp, sp, HAL_INTR_STACK_SIZE
asynchronous_intr:
    /* jump to hal_intr_handler */
    csrrw ra, CSR_JALMNXTI, ra
    csrci    mstatus, MSTATUS_MIE
    /* Decrement nesting counter */
    la      t0, g_hal_intr_nesting
    lw      t1, 0(t0)
    addi    t1, t1, -1
    sw      t1, 0(t0)
    /* if nesting counter != 0 bypass switch to user stack */
    bnez    t1, context_restore
    la      t0, g_hal_intr_stack_frame
    lw      sp, 0(t0)
context_restore:
    /* set previous IE=1 and Machine mode priviledge level */
    li      t0, (MSTATUS_MPP | MSTATUS_MPIE)
    csrs    mstatus, t0
    /* restore mepc and general registers */
    RESTORE_EPC
    RESTORE_CONTEXT
    mret


    .global hal_intr_get_current_vect
hal_intr_get_current_vect:
    csrr    t0, mcause
    li      t1, 0xFFF
    and     a0, t0, t1
    ret

#endif
