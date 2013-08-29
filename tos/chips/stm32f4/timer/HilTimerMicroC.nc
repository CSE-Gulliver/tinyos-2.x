#include "Timer.h"

configuration HilTimerMicroC
{
  provides 
  {
      interface Init;
      interface Timer<TMicro> as TimerMicro[ uint8_t num ];
      interface LocalTime<TMicro>;
  }
}

implementation
{
  components new VirtualizeTimerC(TMicro,uniqueCount(UQ_TIMER_MICRO)) as VirtTimersMicro32;
  components new AlarmToTimerC(TMicro) as AlarmToTimerMicro32;
  components new AlarmMicro32C() as AlarmMicro32;
//  components STM16TIMC;
  
  components LocalTimeMicroC;

  Init = AlarmMicro32;
  TimerMicro = VirtTimersMicro32.Timer;
  LocalTime = LocalTimeMicroC;
  
  VirtTimersMicro32.TimerFrom -> AlarmToTimerMicro32.Timer;
  AlarmToTimerMicro32.Alarm -> AlarmMicro32.Alarm;
}