configuration LocalTimeMicroC
{
	provides interface LocalTime<TMicro>;
}

implementation
{
	
	components CounterTMicro32C;
	components new CounterToLocalTimeC(TMicro);

	LocalTime = CounterToLocalTimeC;
	CounterToLocalTimeC.Counter -> CounterTMicro32C;
}