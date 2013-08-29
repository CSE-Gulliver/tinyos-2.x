#include "Timer.h"

configuration TimerMicroP {
  provides interface Timer<TMicro> as TimerMicro[uint8_t id];
}
implementation {
  components HilTimerMicroC, MainC;
  MainC.SoftwareInit -> HilTimerMicroC;
  TimerMicro = HilTimerMicroC;
}