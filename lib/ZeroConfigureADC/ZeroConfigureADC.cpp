#include <Arduino.h>
/**********************************************************************************
					CONFIGURE  ADC FUNDAMENTAL PARAMETERS 
***********************************************************************************/
int configure_ADC(unsigned long Prescaler,unsigned long SampleNum, unsigned char Samplen)
{
	unsigned long AdjRes;
/*
	0 < Samplen < 63
	Sampling time: time during which the input pin is read
	Sampling time =  (Samplen+1).(CYCLE_CLK/2),   
     0 < Samplen < 63 -> CYCLE_CLK/2 < Sampling time CYCLE_CLK*32
*/
	if (Samplen > 63) return -3;//Samplen Error


switch(SampleNum)
{
	case 0x0://no average
		AdjRes=0x00ul;
	break;
	case 0x1://2 samples Average 
		AdjRes=0x01ul;
	break;
	case 0x2://4 samples Average 
		AdjRes=0x02ul;
	break;
	case 0x3://8 samples Average 
		AdjRes=0x03ul;
	break;
	case 0x4://16 samples average 
		AdjRes=0x04ul;
	break;
	case 0x5://32 samples average 
		AdjRes=0x04ul;
	break;
	case 0x6://Average 64 muestras
		AdjRes=0x04ul;
	break;
	case 0x7://128 samples average 
		AdjRes=0x04ul;
	break;
	case 0x8://256 samples average 
		AdjRes=0x04ul;
	break;
	case 0x9:////512 samples average 
		AdjRes=0x04ul;
	break;
	case 0xA:////1024 samples average 
		AdjRes=0x04ul;
	break;
	default:
		return -2;//SampleNum Error
	break; 
}
	if ( Prescaler==3ul/*48MHz/32 para ADC_CLK < 2.1Mhz*/ || Prescaler==4ul/*48MHz/64*/ 
	|| Prescaler==5ul /*48MHz/128*/|| Prescaler==6ul /*48MHz/256*/|| Prescaler==7ul/*48MHz/512*/ ){}
	else {return -1;} //Prescaler Error
	// Configure CTRLB, AVGCTRL y SAMPCTRL
	ADC->CTRLA.bit.ENABLE = 0; // disable adc
	while( ADC->STATUS.bit.SYNCBUSY == 1 ); // synchronize
	ADC->CTRLB.reg &= 0xF8FF; //Clean prescaler = 000
	ADC->CTRLB.reg |= (Prescaler << 8); 
	ADC->AVGCTRL.reg = (unsigned long) SampleNum |	ADC_AVGCTRL_ADJRES((unsigned long)AdjRes);
	ADC->SAMPCTRL.reg = Samplen; 
	ADC->CTRLA.bit.ENABLE = 1;  // Enable adc
	while( ADC->STATUS.bit.SYNCBUSY == 1 ); // synchronize
return 0;
}
