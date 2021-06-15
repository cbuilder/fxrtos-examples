/**
  ******************************************************************************
  *  @file   start.S
  *  @brief  gd32vf103 startup code.
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

#include "riscv_encoding.h"

    .section .init

//
// All the interrupts are served in FX-RTOS hal_intr_handler
//
vector_base:
    j _start
    .align    2
    .word     0
    .word     0
    .word     _hal_intr_handler_entry
    .word     0
    .word     0
    .word     0
    .word     _hal_intr_handler_entry
    .word     0
    .word     0
    .word     0
    .word     0
    .word     0
    .word     0
    .word     0
    .word     0
    .word     0
    .rept 69
        .word _hal_intr_handler_entry
    .endr

    .globl _start
    .type _start,@function
_start:

    csrc CSR_MSTATUS, MSTATUS_MIE
    /* Jump to logical address first to ensure correct operation of RAM region  */
    la        a0, _start
    li        a1, 1
    slli      a1, a1, 29
    bleu      a1, a0, _start0800
    srli      a1, a1, 2
    bleu      a1, a0, _start0800
    la        a0, _start0800
    add       a0, a0, a1
    jr        a0

_start0800:
    /* Set the the NMI base to share with mtvec by setting CSR_MMISC_CTL */
    li t0, 0x200
    csrs CSR_MMISC_CTL, t0

    /* Intial the mtvt*/
    la t0, vector_base
    csrw CSR_MTVT, t0

    /* Intial the mtvt2 and enable it*/
    la t0, irq_entry
    csrw CSR_MTVT2, t0
    csrs CSR_MTVT2, 0x1

    /* Intial the CSR MTVEC for the Trap ane NMI base addr*/
    la t0, trap_entry
    csrw CSR_MTVEC, t0

#ifdef __riscv_flen
    /* Enable FPU */
    li t0, MSTATUS_FS
    csrs mstatus, t0
    csrw fcsr, x0
#endif

.option push
.option norelax
    la gp, __global_pointer$
.option pop
    la sp, _sp

    /* Load data section */
    la a0, _data_lma
    la a1, _data
    la a2, _edata
    bgeu a1, a2, 2f
  1:
    lw t0, (a0)
    sw t0, (a1)
    addi a0, a0, 4
    addi a1, a1, 4
    bltu a1, a2, 1b
  2:
    /* Clear bss section */
    la a0, __bss_start
    la a1, _end
    bgeu a0, a1, 2f
  1:
    sw zero, (a0)
    addi a0, a0, 4
    bltu a0, a1, 1b
2:
    /* Call global constructors */
    //
    // la a0, __libc_fini_array
    // call atexit
    // call __libc_init_array
    //

    /* argc = argv = 0 */
    li a0, 0
    li a1, 0
    call main
    //
    // tail exit
    //
1:
    j 1b
