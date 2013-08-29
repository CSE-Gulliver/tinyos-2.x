

configuration CounterTMicro32C
{
	provides interface Counter<TMicro,uint32_t>;
	
}

implementation
{
	
	components STM32Micro16TIMC;
	components new TransformCounterC(TMicro,uint32_t,TMicro,uint16_t,0,uint16_t) as TransformCounter;
	
	Counter=TransformCounter;
	TransformCounter.CounterFrom->STM32Micro16TIMC.Counter;
	
}