/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

                                      ---

    A special exception to the GPL can be applied should you wish to distribute
    a combined work that includes ChibiOS/RT, without being obliged to provide
    the source code for any proprietary components. See the file exception.txt
    for full details of how and when the exception can be applied.
*/

/*
 * Imports the Cortex-Mx configuration headers.
 */
#define _FROM_ASM_
#include "chconf.h"
#include "chcore.h"

CONTEXT_OFFSET  EQU     12
SCB_ICSR        EQU     0xE000ED04

                PRESERVE8
                THUMB
                AREA    |.text|, CODE, READONLY

                IMPORT  chThdExit
                IMPORT  chSchDoReschedule
#if CH_DBG_SYSTEM_STATE_CHECK
                IMPORT  dbg_check_unlock
                IMPORT  dbg_check_lock
#endif

/*
 * Performs a context switch between two threads.
 */
                EXPORT _port_switch
_port_switch    PROC
                push    {r4, r5, r6, r7, lr}
                mov     r4, r8
                mov     r5, r9
                mov     r6, r10
                mov     r7, r11
                push    {r4, r5, r6, r7}
                mov     r3, sp
                str     r3, [r1, #CONTEXT_OFFSET]
                ldr     r3, [r0, #CONTEXT_OFFSET]
                mov     sp, r3
                pop     {r4, r5, r6, r7}
                mov     r8, r4
                mov     r9, r5
                mov     r10, r6
                mov     r11, r7
                pop     {r4, r5, r6, r7, pc}
                ENDP

/*
 * Start a thread by invoking its work function.
 * If the work function returns @p chThdExit() is automatically invoked.
 */
                EXPORT  _port_thread_start
_port_thread_start PROC
#if CH_DBG_SYSTEM_STATE_CHECK
                bl      dbg_check_unlock
#endif
                cpsie   i
                mov     r0, r5
                blx     r4
                bl      chThdExit
                ENDP

/*
 * Post-IRQ switch code.
 * Exception handlers return here for context switching.
 */
                EXPORT  _port_switch_from_isr
                EXPORT  _port_exit_from_isr
_port_switch_from_isr PROC
#if CH_DBG_SYSTEM_STATE_CHECK
                bl      dbg_check_lock
#endif
                bl      chSchDoReschedule
_port_exit_from_isr
#if CH_DBG_SYSTEM_STATE_CHECK
                bl      dbg_check_unlock
#endif
                ldr     r2, =SCB_ICSR
                movs    r3, #128
#if CORTEX_ALTERNATE_SWITCH
                lsls    r3, r3, #21
                str     r3, [r2, #0]
                cpsie   i
#else
                lsls    r3, r3, #24
                str     r3, [r2, #0]
#endif
waithere        b       waithere
                ENDP

                END
