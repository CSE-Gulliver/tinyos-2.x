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
 #include "system_stm32f4xx.h"
 
 
module MoteClockP {
  provides 
  {
      interface Init as MoteClockInit;
  }
}

implementation {

  command error_t MoteClockInit.init()
  {
  	GPIO_InitTypeDef GPIO_InitStructure;
  	
  	SystemInit();
  	
  	//GPIOD Periph clock enable
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	//Configure PD12, PD13, PD14 and PD15 in output pushpull mode
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14
			| GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	//GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	
  	/*
    // 1. Cloking the controller from internal HSI RC (8 MHz)
    RCC_HSICmd(ENABLE);

    // wait until the HSI is ready
    while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET);
    RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);

    // 2. Enable ext. high frequency OSC
    RCC_HSEConfig(RCC_HSE_ON);

    // wait until the HSE is ready
    while(RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET);

    // 3. Init PLL
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1,RCC_PLLMul_9); // 72MHz

    //  RCC_PLLConfig(RCC_PLLSource_HSE_Div2,RCC_PLLMul_9); // 36MHz
    RCC_PLLCmd(ENABLE);

    // wait until the PLL is ready
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);

    // 4. Set system clock divders
    RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);
    RCC_ADCCLKConfig(RCC_PCLK2_Div8);
    RCC_PCLK2Config(RCC_HCLK_Div1);
    RCC_PCLK1Config(RCC_HCLK_Div2);
    RCC_HCLKConfig(RCC_SYSCLK_Div1);

    // Flash 1 wait state 
    *(volatile uint32_t *)0x40022000 = 0x12;

    // 5. Clock system from PLL
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    
    */

    return SUCCESS;

  }
 
 
}
