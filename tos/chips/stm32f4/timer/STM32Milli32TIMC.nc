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
 * It provides provides alarm with Milli resolution using the STM32's TIM.
 *
 * @author Tao Li
 */

#include "stm32f4xx_tim.h"
#include "stm32f4xx_it.h"

module STM32Milli32TIMC @safe()
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
    norace uint32_t alarm;
    bool running;
        
    static TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure = 
    {
		.TIM_Period = 1,  
		.TIM_Prescaler = 41016-1, // Down to 1024 Hz (adjust per your clock) with TIM_ClockDivision = TIM_CKD_DIV4
		.TIM_ClockDivision = TIM_CKD_DIV1, 
		.TIM_CounterMode = TIM_CounterMode_Up,	
	};
    
    static void timer_clock_init(void)
    {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	}
    

	static void init_free_timer(void)
	{
		//TIM4 works as a free timer to provide the information of current clock
		//the expiration of the counter means that there is an overflow
		TIM_DeInit(TIM5);
		
		TIM_TimeBaseStructure.TIM_Period=(uint32_t)0xFFFFFFFF;
		
		TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
		TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
	
	}
	
	static void init_alarm_timer(void){
		
		TIM_DeInit(TIM2);
		TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
		
	}
	
    static void init_timer_irq(void){

		NVIC_InitTypeDef conf;

		conf.NVIC_IRQChannel = TIM5_IRQn;
		conf.NVIC_IRQChannelSubPriority = 0;
		conf.NVIC_IRQChannelPreemptionPriority = 1;
		conf.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&conf);
		
		conf.NVIC_IRQChannel = TIM2_IRQn;
		conf.NVIC_IRQChannelSubPriority = 1;
		conf.NVIC_IRQChannelPreemptionPriority = 1;
		conf.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&conf);
		
	}
	
	static void set_alarm_interval(uint32_t interval)
	{

		TIM2 -> CNT = 0;
		TIM2 -> ARR = (uint16_t)interval-1; 
	}
		
    void enableInterrupt()
    {
        /* Enable the TIM Alarm Interrupt */
        atomic {
             TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
             TIM_Cmd(TIM2,ENABLE);
		}           
		running = TRUE;
    }

    void disableInterrupt()
    {
        atomic {
            TIM_Cmd(TIM2,DISABLE);
            TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
		}
         running = FALSE;
              
    }

    command error_t Init.init()
    { 
    	GPIO_ResetBits(GPIOD,GPIO_Pin_15);
     	timer_clock_init();
     		
     	init_timer_irq();
     	init_free_timer();
     	init_alarm_timer();

		atomic {
      		alarm=0;
      	}
      	TIM_Cmd(TIM5,ENABLE);
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
    	
        uint32_t interval;
       	disableInterrupt();
		{
			
			uint32_t now = call Alarm.getNow();
            uint32_t elapsed = now - t0;
            now = TIM5 -> CNT;
            
            if( elapsed >= dt )
            {
                // let the timer expire at the next tic of the TIM!
				interval=2;
//            	atomic
             	alarm = now+2;
//             	atomic system_time = (uint64_t)now+1;
            }
            
            else
            {
                uint32_t remaining = dt - elapsed;
                if( remaining <= 1 )
                {
				   interval=2;
//				 atomic
             	   alarm = now+2;
//                   atomic system_time = (uint64_t)now+1;
                }
                else
                {
					interval = remaining;
//             	   	atomic 
             	   	alarm = now+remaining;
//                	atomic system_time = (uint64_t)(now+remaining);
                }
            }
            
            set_alarm_interval(interval); 
            enableInterrupt();
        }
    }

    async command uint32_t Alarm.getNow()
    {
        uint32_t c;
        c = TIM5->CNT;
        return c;
    }

    async command uint32_t Alarm.getAlarm()
    {
        return alarm;
    }

    async command uint32_t Counter.get()
    {
        return call Alarm.getNow();
    }

    async command bool Counter.isOverflowPending()
    {
        return TIM_GetITStatus(TIM5,TIM_IT_Update);
    }

    async command void Counter.clearOverflow()
    {
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
    }


    async command uint32_t LocalTime.get() {
        return call Alarm.getNow();
    }

    default async event void Counter.overflow() {
        return ;
    }


    void TIM5_IRQHandler(void) @C() @spontaneous(){
    	//entering this interrupt means that there is an overflow of timer
    	static uint32_t j=0;
    	if (TIM_GetFlagStatus(TIM5, TIM_FLAG_Update) != RESET){
    		
          		 j++;
           	if (j==1){
           		GPIO_SetBits(GPIOD,GPIO_Pin_15);
            	
            }
			TIM5->SR = (uint16_t)~TIM_IT_Update;
    		signal Counter.overflow();
    	
    	}
    } 
    
    
    void TIM2_IRQHandler(void) @C() @spontaneous() 
    {
        if (TIM_GetFlagStatus(TIM2, TIM_FLAG_Update) != RESET){
        
            	call Alarm.stop();
            	signal Alarm.fired();
		}
       	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
	
}
