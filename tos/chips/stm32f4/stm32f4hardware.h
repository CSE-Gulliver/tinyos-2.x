/**
 *
 * "Copyright (c) 2009 The Regents of the University  of California.  
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for any purpose, without fee, and without written agreement is
 * hereby granted, provided that the above copyright notice, the following
 * two paragraphs and the author appear in all copies of this software.
 * 
 * IN NO EVENT SHALL THE UNIVERSITY OF CALIFORNIA BE LIABLE TO ANY PARTY FOR
 * DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT
 * OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY OF
 * CALIFORNIA HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE UNIVERSITY OF CALIFORNIA SPECIFICALLY DISCLAIMS ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE PROVIDED HEREUNDER IS
 * ON AN "AS IS" BASIS, AND THE UNIVERSITY OF CALIFORNIA HAS NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS."
 *
 */

/*
 * @author TAO LI
 */

#ifndef STM32F4_HARDWARE_H
#define STM32F4_HARDWARE_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "stm32f4xx_conf.h"
#include "utils.h"

#include "stm32f4xx_it.h"

#define ARM_BASEPRI_INT_MASK (0x000000C0)

typedef uint32_t __nesc_atomic_t;



inline void __nesc_enable_interrupt() {
  uint32_t statusReg = 0;
  
  
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);	  
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);	  
//  __enable_irq();
/*
  if (SysTick_Config(SystemCoreClock / 1000)) {
		// Capture error
		while (1) {};
	}*/
  
/*s
  asm volatile (
           "mrs %0,basepri\n\t"
           "bic %0,%1,#0xc0\n\t"
           "msr basepri, %1"
           : "=r" (statusReg)
           : "0" (statusReg)
           );
           */
  return;
}


inline void __nesc_disable_interrupt() {
  uint32_t statusReg = 0;
  
  
  TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
  TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
/*
  asm volatile (
        "mrs %0,basepri\n\t"
        "orr %0,%1,#0xc0\n\t"
        "msr basepri,%1\n\t"
        : "=r" (statusReg)
        : "0" (statusReg)
        );
        */
  return;
}

//NOTE...at the moment, these functions will ONLY disable the IRQ...FIQ is left alone
inline  __nesc_atomic_t __nesc_atomic_start(void) @spontaneous()
{
  uint32_t result = 0;
//  uint32_t temp = 0;
// CPU_IntDis();
// __set_PRIMASK(1);
// __set_FAULTMASK(1);

 __nesc_disable_interrupt();
// __disable_irq();
   

//asm volatile("" : : : "memory"); // ensure atomic section effect visibility 

/*
  asm volatile (
        "mrs %0,basepri\n\t"
        "orr %1,%2,%4\n\t"
        "msr basepri,%3"
        : "=r" (result) , "=r" (temp)
        : "0" (result) , "1" (temp) , "i" (ARM_BASEPRI_INT_MASK)
        );
  asm volatile("" : : : "memory"); // ensure atomic section effect visibility 
  */
  return result;
}

inline  void __nesc_atomic_end(__nesc_atomic_t oldState) @spontaneous()
{
  uint32_t  statusReg = 0;
//asm volatile("" : : : "memory"); // ensure atomic section effect visibility 
//  __set_PRIMASK(0);
//	__set_FAULTMASK(0);
//  __enable_irq();
/*
  if (SysTick_Config(SystemCoreClock / 1000)) {
		// Capture error
		while (1) {};
  }*/
__nesc_enable_interrupt();
//	__enable_irq();
  
/*
  oldState &= ARM_BASEPRI_INT_MASK;
  asm volatile (
        "mrs %0,basepri\n\t"
        "bic %0, %1, %2\n\t"
        "orr %0, %1, %3\n\t"
        "msr basepri, %1"
        : "=r" (statusReg)
        : "0" (statusReg),"i" (ARM_BASEPRI_INT_MASK), "r" (oldState)
        );
   */     
  return;
}

typedef uint8_t mcu_power_t @combine("mcombine");

/** Combine function.  */
mcu_power_t mcombine(mcu_power_t m1, mcu_power_t m2) {
      return (m1 < m2)? m1: m2;
}


#endif