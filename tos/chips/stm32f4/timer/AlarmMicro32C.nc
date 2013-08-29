generic configuration AlarmMicro32C(){
	
	provides interface Init;
  	provides interface Alarm<TMicro,uint32_t>;
	
}

implementation{
	components STM16TIMC;	
	components CounterTMicro32C;
	components new TransformAlarmC(TMicro,uint32_t,TMicro,uint16_t,0) as TransformAlarm;
	
	Init=STM16TIMC;
	Alarm=TransformAlarm;
	TransformAlarm.AlarmFrom -> STM16TIMC.Alarm;
	TransformAlarm.Counter->CounterTMicro32C;
	
}