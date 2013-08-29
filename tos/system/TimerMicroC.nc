#include "Timer.h"

generic configuration TimerMicroC() {
  provides interface Timer<TMicro>;
}
implementation {
  components TimerMicroP;

  // The key to unique is based off of TimerMilliC because TimerMilliImplP
  // is just a pass-through to the underlying HIL component (TimerMilli).
  Timer = TimerMicroP.TimerMicro[unique(UQ_TIMER_MICRO)];
}