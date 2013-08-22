/**
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

/**
 * @author Thomas Schmid
 */

#include "hardware.h"
#include "stm32f4xx_rcc.h"



module PlatformP {
    provides {
        interface Init;
        interface PlatformReset;
    }
    uses {
        interface Init as MoteInit;
        interface Init as MoteClockInit;
        interface Init as McuSleepInit;
//        interface HplSTM32Interrupt as Interrupt;
    }
}
implementation {

    command error_t Init.init() {
        // Initialize the gpio pints for lowest state possible
        GPIO_InitTypeDef GPIO_InitStructure;
        
//        USART_InitTypeDef USART_InitStructure;
        
   	  	// GPIOD Periph clock enable
//		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//		GPIO_Init(GPIOD, &GPIO_InitStructure);
    	/*    
        GPIO_InitTypeDef gpioa = {
            (uint16_t) 0xFFFF, // select all except TMS, TCK and TDI
            GPIO_Speed_2MHz,
//            GPIO_Mode_Out_OD // open draine mode
            GPIO_Mode_Out // open draine mode
        };
		*/
        GPIO_Init(GPIOA, &GPIO_InitStructure);
        GPIOA->ODR = 0;
        GPIO_Init(GPIOB, &GPIO_InitStructure);
        GPIOB->ODR = 0;
        GPIO_Init(GPIOC, &GPIO_InitStructure);
        GPIOC->ODR = 0;
        GPIO_Init(GPIOD, &GPIO_InitStructure);
        GPIOD->ODR = 0;
        
        *NVIC_CCR = *NVIC_CCR | 0x200; /* Set STKALIGN in NVIC */
		
        // Init clock system
        call MoteClockInit.init();
        call McuSleepInit.init();
        
        //I am not sure how to configure it now. But now, the GPIOs are configured as output
//        RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
//        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOA, ENABLE);
//        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

        call MoteInit.init();


        return SUCCESS;
    }

    async command void PlatformReset.reset() {
        while (1);
        return; // Should never get here.
    }

    void nmi_handler(void)
    {
        while(1) {};
        return ;
    }

    void hardfault_handler(void)
    {
        while(1) {};
        return ;
    }

    //Functions definitions
    void myDelay(unsigned long delay )
    {
        while(delay) delay--;
    }

//    async event void Interrupt.fired(void)
//{

//}

}

