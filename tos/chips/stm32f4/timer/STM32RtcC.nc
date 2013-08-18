/**
 * "Copyright (c) 2009 The Regents of the University of California.
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for any purpose, without fee, and without written agreement
 * is hereby granted, provided that the above copyright notice, the following
 * two paragraphs and the author appear in all copies of this software.
 *
 * IN NO EVENT SHALL THE UNIVERSITY OF CALIFORNIA BE LIABLE TO ANY PARTY FOR
 * DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT
 * OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY
 * OF CALIFORNIA HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * THE UNIVERSITY OF CALIFORNIA SPECIFICALLY DISCLAIMS ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE PROVIDED HEREUNDER IS
 * ON AN "AS IS" BASIS, AND THE UNIVERSITY OF CALIFORNIA HAS NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS."
 */

/**
 * STM32Alarm provides an alarm using the STM32's RTC.
 *
 * @author Thomas Schmid
 */

#include "stm32f4xx_rtc.h"

module STM32RtcC @safe()
{
    provides {
        interface Init;
        interface Alarm<TMilli,uint32_t> as Alarm;
        interface Counter<TMilli,uint32_t> as Counter;
        interface LocalTime<TMilli> as LocalTime;
    }
}
implementation
{

    norace uint32_t last_interval;
    norace uint32_t system_time;
    bool running;
    
    volatile uint32_t wake_up_interval;
    
    void init_rtc();
    void init_rtc_exti();
    
    static NVIC_InitTypeDef NVIC_InitStructure={
	
		.NVIC_IRQChannel = RTC_WKUP_IRQn,
		.NVIC_IRQChannelPreemptionPriority = 1,
		.NVIC_IRQChannelSubPriority = 0,
		.NVIC_IRQChannelCmd = ENABLE,		
		
	};

	void init_rtc() {
		RTC_InitTypeDef RTC_InitStructure;

		/* Enable the PWR clock */
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

		/* Allow access to RTC */
		PWR_BackupAccessCmd(ENABLE);
		/* Reset BKP Domain */
		RCC_BackupResetCmd(ENABLE);
		RCC_BackupResetCmd(DISABLE);

		/* Enable the LSI OSC */
		RCC_LSICmd(ENABLE);

		/* Wait till LSI is ready */
		while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
			;

		/* Select the RTC Clock Source */
		RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);

		/* Enable the RTC Clock */
		RCC_RTCCLKCmd(ENABLE);
	
		/* Wait for RTC APB registers synchronisation */
		RTC_WaitForSynchro();

		/* Configure the RTC data register and RTC prescaler */
		RTC_InitStructure.RTC_AsynchPrediv = 0x20;
		RTC_InitStructure.RTC_SynchPrediv = 0x00;
		RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;
		RTC_Init(&RTC_InitStructure);
	}

	
	void init_rtc_exti(){
		EXTI_InitTypeDef EXTI_InitStructure;

		/* Connect EXTI_Line22 to the RTC Wakeup event */
		EXTI_ClearITPendingBit(EXTI_Line22);
		EXTI_InitStructure.EXTI_Line = EXTI_Line22;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure);

		RTC_WakeUpClockConfig(RTC_WakeUpClock_CK_SPRE_16bits);

	}


	//I think we can delete RTC_WaitForLastTask statements because
	//1. they are in atomic scopes
	//2. stm32f4xx_rtc.c implements some protective mechanisms in each function. 
	//	Notice: there is no such mechanism in RTC_ITConfig, RTC_ClearITPendingBit (I found two so far)	
	
	
	//Please call init_rtc_wu_second(uint32_t interval) before you enable the interrupt; otherwise
	//you cannot start the wake up interrupt
    void enableInterrupt()
    {
        /* Enable the RTC Alarm Interrupt */
        atomic {
//            RTC_ITConfig(RTC_IT_ALR, ENABLE);
            RTC_ITConfig(RTC_IT_WUT, ENABLE);
            RTC_WakeUpCmd(ENABLE);
//            RTC_WaitForLastTask();
            running = TRUE;
        }
    }

    void disableInterrupt()
    {
        /* Disable the RTC Alarm Interrupt */
        atomic {
//            RTC_ClearITPendingBit(RTC_IT_ALR);
//            RTC_ClearITPendingBit(RTC_Alarm_IRQn);
//			 GPIO_ToggleBits(GPIOD, GPIO_Pin_14);
//            RTC_WaitForLastTask();
              RTC_ITConfig(RTC_IT_WUT, DISABLE);
              RTC_WakeUpCmd(DISABLE);
//            RTC_WaitForLastTask();
              running = FALSE;
        }
    }


    command error_t Init.init()
    {
     	init_rtc();
     	init_rtc_exti();
		
		atomic {
      		system_time=0;  
      		last_interval=0;

      	}
		
        return SUCCESS;
    }

    async command void Alarm.start( uint32_t dt )
    {
        call Alarm.startAt( call Alarm.getNow(), dt );
    }

    async command void Alarm.stop()
    {
        disableInterrupt();
    }

    async command bool Alarm.isRunning()
    {
        return running;
    }

    async command void Alarm.startAt( uint32_t t0, uint32_t dt )
    {
       	disableInterrupt();
		NVIC_Init(&NVIC_InitStructure);
		{
            uint32_t now = call Alarm.getNow();
            uint32_t elapsed = now - t0;
//            now = RTC_GetCounter();
            now = system_time+last_interval-RTC_GetWakeUpCounter();
            if( elapsed >= dt )
            {
                // let the timer expire at the next tic of the RTC!
                
             	wake_up_interval=0x00;
                atomic system_time = now+1;
//                RTC_WaitForLastTask();
            }
            else
            {
                uint32_t remaining = dt - elapsed;
                if( remaining <= 1 )
                {
                   wake_up_interval=0x00;
                    atomic system_time = now+1;
//                    RTC_WaitForLastTask();
                }
                else
                {
//                    RTC_SetAlarm(now+remaining); 
                  	wake_up_interval=remaining; 
                    atomic system_time = now+remaining;
//                    RTC_WaitForLastTask();
                }
            }
            RTC_SetWakeUpCounter(wake_up_interval); 
            last_interval=wake_up_interval;
            enableInterrupt();
        }
    }

    async command uint32_t Alarm.getNow()
    {
        uint32_t c;
       	c = system_time+last_interval-RTC_GetWakeUpCounter();
        return c;
    }

    async command uint32_t Alarm.getAlarm()
    {
        return system_time;
    }

    async command uint32_t Counter.get()
    {
        return call Alarm.getNow();
    }

    async command bool Counter.isOverflowPending()
    {
        return (system_time<0);
    }

    async command void Counter.clearOverflow()
    {
//        RTC_ClearITPendingBit(RTC_IT_OW);
//        RTC_WaitForLastTask();
		system_time=0;
    }

    async command uint32_t LocalTime.get() {
        return call Alarm.getNow();
    }


    default async event void Counter.overflow() {
        return;
    }

    /**
     * This is the interrupt handler defined in stm32-vectors.c.
     */
     
    void RTC_WKUP_IRQHandler(void) @C() @spontaneous() 
    {
    	    
        if (RTC_GetITStatus(RTC_IT_WUT) != RESET)
        {
            // interrupt gets cleared when the timer is stopped in
            // Alarm.stop()
            RTC_ClearITPendingBit(RTC_IT_WUT);
	   		EXTI_ClearITPendingBit(EXTI_Line22);
	   		
            call Alarm.stop();
            signal Alarm.fired();
        } 
       
        if (system_time<0)
        {
//            RTC_ClearITPendingBit(RTC_IT_OW);
//            RTC_WaitForLastTask();
            signal Counter.overflow();
        }

    }

}

