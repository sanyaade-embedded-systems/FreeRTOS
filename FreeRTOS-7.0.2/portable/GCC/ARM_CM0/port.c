/*
    FreeRTOS V7.0.1 - Copyright (C) 2011 Real Time Engineers Ltd.


    ***************************************************************************
     *                                                                       *
     *    FreeRTOS tutorial books are available in pdf and paperback.        *
     *    Complete, revised, and edited pdf reference manuals are also       *
     *    available.                                                         *
     *                                                                       *
     *    Purchasing FreeRTOS documentation will not only help you, by       *
     *    ensuring you get running as quickly as possible and with an        *
     *    in-depth knowledge of how to use FreeRTOS, it will also help       *
     *    the FreeRTOS project to continue with its mission of providing     *
     *    professional grade, cross platform, de facto standard solutions    *
     *    for microcontrollers - completely free of charge!                  *
     *                                                                       *
     *    >>> See http://www.FreeRTOS.org/Documentation for details. <<<     *
     *                                                                       *
     *    Thank you for using FreeRTOS, and thank you for your support!      *
     *                                                                       *
    ***************************************************************************


    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
    >>>NOTE<<< The modification to the GPL is included to allow you to
    distribute a combined work that includes FreeRTOS without being obliged to
    provide the source code for proprietary components outside of the FreeRTOS
    kernel.  FreeRTOS is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
    or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
    more details. You should have received a copy of the GNU General Public
    License and the FreeRTOS license exception along with FreeRTOS; if not it
    can be viewed here: http://www.freertos.org/a00114.html and also obtained
    by writing to Richard Barry, contact details for whom are available on the
    FreeRTOS WEB site.

    1 tab == 4 spaces!

    http://www.FreeRTOS.org - Documentation, latest information, license and
    contact details.

    http://www.SafeRTOS.com - A version that is certified for use in safety
    critical systems.

    http://www.OpenRTOS.com - Commercial support, development, porting,
    licensing and training services.
*/

/*-----------------------------------------------------------
 * Implementation of functions defined in portable.h for the ARM CM0 port.
 *----------------------------------------------------------*/

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"


/*
 * M0 has two stack pointers:
 * - MSP ('Main' Stack Pointer), used during exceptions, and
 * - PSP ('Process' Stack Pointer), used during "thread" or "normal" mode.
 *
 * MSP ('Main' Stack Pointer)
 * Out of reset the MSP is loaded with the initial stack pointer
 * located in the first entry of the vector table located at address 0x00000000.
 * Handler mode (processing any interupt or exception) ALWAYS uses MSP.
 * For a fault to be trapped properly, MSP must point to a safe stack area.
 *
 * PSP ('Process' Stack Pointer)
 * Normal processing uses PSP ('Process' Stack Pointer), in "thread mode" or "normal mode".
 * FreeRTOS tasks run using PSP.
 *
 * FreeRTOS task stacks are allocated from the heap using using
 * "portMalloc", so they are independent of original reset stack.
 * 
 * MSP is set to the original reset-stack, which reuses the stack
 * that main() was using. Anything on the original reset-stack used by
 * "main" will be clobbered.
 *
 * The control register bit[1] is set to 1 (use PSP) before starting the first task.
 */


#ifndef configKERNEL_INTERRUPT_PRIORITY
  #error configKERNEL_INTERRUPT_PRIORITY must be defined in FreeRTOSConfig.h - suggest 2
#endif

/* Constants required to set up the initial stack. */
#define portINITIAL_XPSR            ( 0x01000000 )  // Set the T bit in the Combined Program Status Register (XPSR) to execute in thumb state.

/* The priority used by the kernel is assigned to a variable to make access
from inline assembler easier. */
const unsigned long ulKernelPriority = configKERNEL_INTERRUPT_PRIORITY;

/* Each task maintains its own interrupt status in the critical nesting
variable. */
static unsigned portBASE_TYPE uxCriticalNesting = 0xaaaaaaaa;

/*
 * Exception handlers.
 */
void xPortPendSVHandler( void ) __attribute__ (( naked ));
void xPortSysTickHandler( void );
void vPortSVCHandler( void ) __attribute__ (( naked ));

/*
 * Start first task is a separate function so it can be tested in isolation.
 */
static void vPortStartFirstTask( void ) __attribute__ (( naked ));

/*-----------------------------------------------------------*/

/*
 * See header file for description.
 */
portSTACK_TYPE *pxPortInitialiseStack( portSTACK_TYPE *pxTopOfStack, pdTASK_CODE pxCode, void *pvParameters )
{
    /* Simulate stack frame as it would be pushed onto stack by a context switch interrupt. */
    pxTopOfStack--;                             /* Decrement stack pointer. */
    *pxTopOfStack = portINITIAL_XPSR;           /* Store xPSR Program Status Register. */
    pxTopOfStack--;                             /* Decrement stack pointer. */
    *pxTopOfStack = ( portSTACK_TYPE ) pxCode;  /* Return address (new PC) */
    pxTopOfStack--;                             /* Decrement stack pointer. */
    *pxTopOfStack = 0;                          /* LR (R14)*/
    pxTopOfStack -= 5;                          /* R12, R3, R2, R1. */
    *pxTopOfStack = ( portSTACK_TYPE ) pvParameters; /* R0 */
    pxTopOfStack -= 8;                          /* R11, R10, R9, R8, R7, R6, R5 and R4. */
    return pxTopOfStack;
}
/*-----------------------------------------------------------*/

// Called only once by startFirstTask (naked function)
// Note: pxPortInitialiseStack has placed a dummy full stack frame on each task stack
void vPortSVCHandler( void )
{
  __asm volatile (
    ".syntax unified        \n"
    // Restore the context.
    " ldr r3, pxCurrentTCBConst2    \n" /* Get the TCB pointer address */
    " ldr r1, [r3]                  \n" /* Load the address of the current TCB */
    " ldr r0, [r1]                  \n" /* The first item in pxCurrentTCB is the task top of stack. */
    // Pop the registers that are not automatically saved on exception entry and the critical nesting count.
    " ldm r0!, {r4-r7}              \n" /* Pop r8-r11 */
    " mov r8,  r4                   \n"
    " mov r9,  r5                   \n"
    " mov r10, r6                   \n"
    " mov r11, r7                   \n"
    " ldm r0!, {r4-r7}              \n" /* Pop r4-r7  */
    " msr psp, r0                   \n" // Set the new stack pointer for the current task
    // Set the link register R14 to special value 0xFFFFFFFD, which tells the processor to
    // perform "return from exception" processing using Thread mode (ie, state information
    // is stored on PSP stack pointer and execution will continue using PSP)
    // Cleaner code could just load -3 and set r14 (T2: mov r3,#03; neg r3,r3; mov r14,r3)...     
    " mov r1,  r14                  \n" // Copy LR (already has one of the 'special' exception values 0xFFFFFFF? within exception)
    " movs r2, #0xD                 \n" // prepare to switch to thread mode for current task, use PSP
    " orrs r1, r1, r2               \n" // or in "D" to low 4 bits
    " mov r14, r1                   \n" // set LR to 0xFFFFFFFD
    " bx r14                        \n" // exception return to start task
    " .align 2                      \n"
    "pxCurrentTCBConst2: .word pxCurrentTCB \n"
    ".syntax divided        \n"
  );
}
/*-----------------------------------------------------------*/

static void vPortStartFirstTask( void )
{
  __asm volatile (
    // Next two lines load reset stack pointer value into r0
    // (&_vStackTop, see freertos_cr_startup_lpc11.c)
    " ldr r0, =0x00   \n" /* cm0 has fixed vector table at address 0x00000000 */
    " ldr r0, [r0]    \n" /* Fetch the first entry, the initial stack pointer. */
    " msr msp, r0     \n" /* Set MSP to original top of stack (will invalidate original stack contents) */
    " msr psp, r0     \n" /* Set PSP to original top of stack (will invalidate original stack contents) */
    " ldr r0, =0x02   \n" /* Use PSP for stack pointer (bit[1] of the control register sets stack pointer use: 0=msp 1=psp) */
    " msr control, r0 \n" /* Switch to Process Stack Pointer (PSP) */
    " cpsie i         \n" /* Enable interrupts (clear PRIMASK bit) */
    " svc 0           \n" /* Issue an SVC exception Supervisor Call to vPortSVCHandler() to start first task. */
    " .align 2        \n"
  );
}
/*-----------------------------------------------------------*/

/*
 * See header file for description.
 */
portBASE_TYPE xPortStartScheduler( void )
{
    // Start the timer that generates the tick ISR.  Interrupts are disabled here already.
    // NOTE: CMSIS SysTick_Config function sets SysTick exception priority to the lowest possible value.
    // However, we add an explicit initialization below to make this visible.
    SysTick_Config(configCPU_CLOCK_HZ / configTICK_RATE_HZ);

    /* Make PendSV, SysTick, and SVC the same priority as the kernel. */
    NVIC_SetPriority(SysTick_IRQn, configKERNEL_INTERRUPT_PRIORITY);
    NVIC_SetPriority(PendSV_IRQn, configKERNEL_INTERRUPT_PRIORITY);
    NVIC_SetPriority(SVCall_IRQn, configKERNEL_INTERRUPT_PRIORITY);

    /* Initialize the critical nesting count ready for the first task. */
    uxCriticalNesting = 0;

    /* Start the first task. */
    vPortStartFirstTask();  /* interrupts get enabled within this call */

    /* Should not get here! */
    return 0;
}
/*-----------------------------------------------------------*/

void vPortEndScheduler( void )
{
  /* It is unlikely that the CM0 port will require this function as there
    is nothing to return to.  */
}
/*-----------------------------------------------------------*/

void vPortYieldFromISR( void )
{
    /* Set a PendSV to request a context switch. */
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
}
/*-----------------------------------------------------------*/

void vPortEnterCritical( void )
{
    portDISABLE_INTERRUPTS();
    uxCriticalNesting++;
}
/*-----------------------------------------------------------*/

void vPortExitCritical( void )
{
    uxCriticalNesting--;
    if( uxCriticalNesting == 0 )
    {
        portENABLE_INTERRUPTS();
    }
}
/*-----------------------------------------------------------*/

// Perform task context switch (this is a naked function)
void xPortPendSVHandler( void )
{
  __asm volatile (
    ".syntax unified        \n"
    " mrs r0, psp                   \n" /* Fetch PSP ('Process' Stack Pointer) into R0 */
    " ldr r3, pxCurrentTCBConst     \n" /* Get the location of the TCB pointer into r3 */
    " ldr r2, [r3]                  \n" /* r2 is the address of first member of TCB, the task's stack */
    // Push the full task context (r4-r11) onto task's stack (PSP)
    " subs r0, #4                   \n"
    " str r7, [r0]                  \n" /* Save r7 */
    " subs r0, #4                   \n"
    " str r6, [r0]                  \n" /* Save r6 */
    " subs r0, #4                   \n"
    " str r5, [r0]                  \n" /* Save r5 */
    " subs r0, #4                   \n"
    " str r4, [r0]                  \n" /* Save r4 */
    " mov r4, r8                    \n" /* Prepare the remaining context frame */
    " mov r5, r9                    \n" 
    " mov r6, r10                   \n"
    " mov r7, r11                   \n" 
    " subs r0, #4                   \n"
    " str r7, [r0]                  \n" /* Save r11 */
    " subs r0, #4                   \n"
    " str r6, [r0]                  \n" /* Save r10 */
    " subs r0, #4                   \n"
    " str r5, [r0]                  \n" /* Save r9 */
    " subs r0, #4                   \n"
    " str r4, [r0]                  \n" /* Save r8 */
    "                               \n"
    " str r0, [r2]                  \n" /* Save the new task top of stack (PSP) into the first member of the TCB */
    " push {r14}                    \n" /* save link register r14 on the main (exception) stack MSP */
    "                               \n" /* LR has special value since we are within an exception */
    " cpsid i                       \n" /* Disable all interrupts except NMI (set PRIMASK) */
    " bl vTaskSwitchContext         \n" /* Call vTaskSwitchContext using MSP since we are in exception handler */
    " cpsie i                       \n" /* Enable interrupts (clear PRIMASK) */
    // restore link register R14 value: pop {r14}
    " ldr r3, [SP]                  \n" // get back LR from MSP, adjust MSP
    " add SP, #4                    \n"
    " mov r14, r3                   \n"
    // Restore the context, including the critical nesting count.
    " ldr r3, pxCurrentTCBConst     \n" /* Get back the location of the TCB pointer into r3 */
    " ldr r1, [r3]                  \n" /* r1 holds the address of first member of TCB, the (possibly new) task stack pointer */
    " ldr r0, [r1]                  \n" /* Load r0 with the first item in pxCurrentTCB, the tasks top of stack. */
    // Load the new full context from the PSP stack
    " ldm r0!, {r4-r7}              \n" /* Pop r8-r11 */
    " mov r8,  r4                   \n" 
    " mov r9,  r5                   \n" 
    " mov r10, r6                   \n" 
    " mov r11, r7                   \n" 
    " ldm r0!, {r4-r7}              \n" /* Pop r4-r7 */
    " msr psp, r0                   \n" /* Restore the task stack pointer (PSP) */
    " bx r14                        \n" /* and return from exception */
    "                               \n" 
    " .align 2                      \n" 
    "pxCurrentTCBConst: .word pxCurrentTCB    \n"
    "                                         \n"
    ".syntax divided        \n"
  );
}
/*-----------------------------------------------------------*/

void xPortSysTickHandler( void )
{
    unsigned long ulDummy;

    /* If using preemption, also force a context switch. */
    #if configUSE_PREEMPTION == 1
        SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
    #endif

    ulDummy = portSET_INTERRUPT_MASK_FROM_ISR();
    {
        vTaskIncrementTick();
    }
    portCLEAR_INTERRUPT_MASK_FROM_ISR( ulDummy );
}
/*-----------------------------------------------------------*/
