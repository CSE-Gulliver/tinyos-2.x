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
 * It provides provides alarm with Macro granuality using the STM32's TIM.
 *
 * @author Tao Li
 */

#include "stm32f4xx_tim.h"
#include "stm32f4xx_it.h"

module STM16TIMC @safe()
{
    provides {
        interface Init;
        interface Alarm<TMicro,uint16_t> as Alarm;
        interface Counter<TMicro,uint16_t> as Counter;
//        interface LocalTime<TMilli> as LocalTime;
    }
}

implementation
{
//    norace uint32_t last_interval;
    norace uint16_t alarm;
//    norace uint16_t current_system_time;
//   	norace uint32_t previous_system_time;
    bool running;
    bool alarm_set;
//    bool overflow;
        
    static TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure = {
		
		.TIM_Period = 1,  
		.TIM_Prescaler = 80 - 1, // Down to 1050000 Hz ~= 1048576 Hz (adjust per your clock)
		.TIM_ClockDivision = TIM_CKD_DIV1,
		.TIM_CounterMode = TIM_CounterMode_Up,	
	
	};
	
	static NVIC_InitTypeDef NVIC_InitStructure={
		
		.NVIC_IRQChannel = TIM2_IRQn,
		.NVIC_IRQChannelPreemptionPriority = 0,
		.NVIC_IRQChannelSubPriority = 0,
		.NVIC_IRQChannelCmd = ENABLE,		
	
	};
    
    static void timer_clock_init(void)
    {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	}
    

	static void init_free_timer(void){
		//TIM3 works as a free timer to provide the information of current clock
		//the expiration of the counter means that there is an overflow
		TIM_DeInit(TIM2);
		TIM_TimeBaseStructure.TIM_Period=(uint16_t)0xFFFF;
		
		TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
//		TIM_ARRPreloadConfig(TIM3, ENABLE);
		
		TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	
	}
	
	static void init_alarm_timer(void){
		
		uint32_t system_frequency = 1050000;
		
		//the resolution of the timer is 1s/20000=50us
		uint32_t micro_timer_frequency = 21000;
		
		TIM_DeInit(TIM3);
//		TIM_TimeBaseStructure.TIM_Period=1;
//		TIM_TimeBaseStructure.TIM_Period = system_frequency / micro_timer_frequency;
//		TIM_TimeBaseStructure.TIM_Period=(uint32_t)1000000/2;
		
		TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
//		TIM_ARRPreloadConfig(TIM2, ENABLE);
//		TIM_ClearFlag(TIM2,TIM_FLAG_Update);
//		TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
		
	}
	
    static void init_timer_irq(void){
//		NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn;

		NVIC_InitTypeDef conf;
		conf.NVIC_IRQChannel = TIM2_IRQn;
		conf.NVIC_IRQChannelSubPriority = 1;
		conf.NVIC_IRQChannelPreemptionPriority = 1;
		conf.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&conf);
		
		conf.NVIC_IRQChannel = TIM3_IRQn;
		conf.NVIC_IRQChannelSubPriority = 0;
		conf.NVIC_IRQChannelPreemptionPriority = 0;
		conf.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&conf);
		
//		conf.NVIC_IRQChannel = TIM3_IRQn;
//		conf.NVIC_IRQChannelSubPriority = 1;
//		NVIC_Init(&conf);
		
	}
	
	static void set_alarm_interval(uint16_t interval){
//		TIM_DeInit(TIM3);

//		TIM_TimeBaseStructure.TIM_Period = (uint16_t)interval - 1;
//		TIM_TimeBaseStructure.TIM_Prescaler = 80 - 1; // Down to 1 MHz (adjust per your clock)
//		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
//		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	
//		TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

		TIM3 -> CNT = 0;
		TIM3 -> ARR = (uint16_t)interval-1; 
	}
		
    void enableInterrupt()
    {
        /* Enable the TIM Alarm Interrupt */
//        atomic {
//   		   TIM_SelectOnePulseMode(TIM2,TIM_OPMode_Single);
//		   	   TIM_ARRPreloadConfig(TIM2, ENABLE);
             TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
             TIM_Cmd(TIM3,ENABLE);
//		}
           running = TRUE;
    }

    void disableInterrupt()
    {
//        atomic {
        	
            
            TIM_Cmd(TIM3,DISABLE);
            TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
//              TIM_DeInit(TIM2);
//		}
         running = FALSE;
              
    }

    command error_t Init.init()
    { 
    	GPIO_ResetBits(GPIOD,GPIO_Pin_15);
     	timer_clock_init();
     		
     	init_timer_irq();
     	init_free_timer();
     	init_alarm_timer();
    
//     	set_alarm_interval(1000000/2);
//     	TIM_ARRPreloadConfig(TIM2, ENABLE);
     	
//     	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
//      TIM_Cmd(TIM2,ENABLE);

		atomic {
      		alarm=0;
//      		alarm_set=FALSE;
//      		overflow=FALSE;
      	}
//      	TIM_Cmd(TIM2,ENABLE);
      	TIM_Cmd(TIM2,ENABLE);
//        GPIO_ToggleBits(GPIOD,GPIO_Pin_15);
        return SUCCESS;
    }

    async command void Alarm.start( uint16_t dt )
    {
//    	GPIO_SetBits(GPIOD,GPIO_Pin_15);
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

    async command void Alarm.startAt( uint16_t t0, uint16_t dt )
    {
    	
        uint16_t interval;
//    	alarm_set = FALSE;
       	disableInterrupt();
//       	NVIC_Init(&NVIC_InitStructure);
		{
			
			uint16_t now = call Alarm.getNow();
            uint16_t elapsed = now - t0;
            now = TIM2 -> CNT;
            
            if( elapsed >= dt )
            {
                // let the timer expire at the next tic of the TIM!
//             	wake_up_interval=1;
				interval=5;
//             	atomic 
             	alarm = now+5;
//             	atomic system_time = (uint64_t)now+1;
				
            }
            
            else
            {
                uint16_t remaining = dt - elapsed;
                if( remaining <= 1 )
                {
//                   wake_up_interval=1;
				   interval=5;
//             	   atomic 
             	   alarm = now+5;
//                   atomic system_time = (uint64_t)now+1;
                }
                else
                {
//                  	wake_up_interval=remaining; 
					interval = remaining;
//             	   	atomic 
             	   	alarm = now+remaining;
             	   	
//                	atomic system_time = (uint64_t)(now+remaining);
                }
            }
            
//            alarm_set=TRUE;
//			GPIO_ToggleBits(GPIOD,GPIO_Pin_15);
            set_alarm_interval(interval); 
//            last_interval=wake_up_interval;
            enableInterrupt();

        }
    }

    async command uint16_t Alarm.getNow()
    {
        	uint16_t c;
//    	TIM_Cmd(TIM3,DISABLE);
        c = TIM2->CNT;
//        TIM_Cmd(TIM3,ENABLE);
        return c;
    }

    async command uint16_t Alarm.getAlarm()
    {
        return alarm;
    }

    async command uint16_t Counter.get()
    {
    	
//    	uint16_t c;
//    	TIM_Cmd(TIM3,DISABLE);
//        c = TIM2->CNT;
//        TIM_Cmd(TIM3,ENABLE);
//        return c;
        return call Alarm.getNow();
    }

    async command bool Counter.isOverflowPending()
    {
//        return (system_time<0);
        return TIM_GetITStatus(TIM2,TIM_IT_Update);
    }

    async command void Counter.clearOverflow()
    {
    	
//    	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
//        RTC_ClearITPendingBit(RTC_IT_OW);
//        RTC_WaitForLastTask();
//		system_time=0;

		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    }

/*
    async command uint32_t LocalTime.get() {
        return call Alarm.getNow();
    }
*/
    default async event void Counter.overflow() {
        return ;
    }
//    
    void TIM2_IRQHandler(void) @C() @spontaneous(){
//    	if (TIM_GetFlagStatus(TIM2, TIM_FLAG_Update) != RESET)
    	//entering this interrupt means that there is an overflow of timer
    	
    	if (TIM_GetFlagStatus(TIM2, TIM_FLAG_Update) != RESET){
    		static uint32_t j=0;
          		 j++;
           	if (j%10==0){
           		GPIO_ToggleBits(GPIOD,GPIO_Pin_14);
            	
            }
			TIM2->SR = (uint16_t)~TIM_IT_Update;
    		signal Counter.overflow();
    	
    	}
//    	
    } 
    
    
    void TIM3_IRQHandler(void) @C() @spontaneous() 
    {
//    	 GPIO_ToggleBits(GPIOD,GPIO_Pin_15);
//        uint16_t current_system_time;
//        static uint32_t i=0;
        
//        TIM_Cmd(TIM3,DISABLE);
//        current_system_time = TIM3->CNT;
//        TIM_Cmd(TIM3,ENABLE);
        
        if (TIM_GetFlagStatus(TIM3, TIM_FLAG_Update) != RESET){
        
            // interrupt gets cleared when the timer is stopped in
            // Alarm.stop()
            
//           if (current_system_time>=alarm && alarm_set==TRUE){
           		
            	call Alarm.stop();
            	signal Alarm.fired();
//            	GPIO_ToggleBits(GPIOD,GPIO_Pin_13);
            	
            		
//       		}
		}
       	   TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    }
	
}

