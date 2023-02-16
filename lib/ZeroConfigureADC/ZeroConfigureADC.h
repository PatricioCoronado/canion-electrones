//#ifndef ZEROCONFIGUREADC_H_INCLUDED
//#define ZEROCONFIGUREADC_H_INCLUDED
//#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
//#else
//#include "WProgram.h"
//#endif
/********************************************************************************
					CONFIGURE  ADC FUNDAMENTAL PARAMETERS 
establishes 3 fundamental parameters for the acquisition speed.

-Prescaler:divide GCLK_ADC=48MHz between DIV.
CLK_ADC=GCLK/DIV
Precaler = 0 DIV=4 	//Not used in this library becouse CLK_ADCmax=2.1MHz
Precaler = 1 DIV=8	//Not used in this library becouse CLK_ADCmax=2.1MHz
Precaler = 2 DIV=16	//Not used in this library becouse CLK_ADCmax=2.1MHz
Precaler = 3 DIV=32
Precaler = 4 DIV=64
Precaler = 5 DIV=128
Precaler = 6 DIV=256
Precaler = 7 DIV=512

-SamplenNum: number of samples averaged in each acquisition
SamplenNum = 0, samples averaged=1
SamplenNum = 1, samples averaged=2
SamplenNum = 2, samples averaged=4
SamplenNum = 3, samples averaged=8
SamplenNum = 4, samples averaged=18
SamplenNum = 5, samples averaged=32
SamplenNum = 6, samples averaged=64
SamplenNum = 7, samples averaged=128
SamplenNum = 8, samples averaged=256
SamplenNum = 9, samples averaged=512
SamplenNum = 10, samples averaged=1024

-Samplen: It determines the number of cycles during which the input pin 
is read. So set time during which the input pin is read = "Sampling time"
Sampling time =  (Samplen+1).(CYCLE_CLK/2). CYCLE_CLK depends on Prescaler.
Why do this?
The analog input of the ADC has a capacity of 3.3pF and a series resistance
of 3.3Koh. To do a good measure you need to charge that capacity. The time 
required for that depends on the source output resistance. If it is high you
need more clk cycles to load the input capacity.
t_charge=5*(3.3e3 + Rs)*3.3e-12 (charging 99%)
And the time of a CLK_ADC cycle is T_CLK_ADC = Prescaler/(48e6)
Are needed t_charge/T_CLK cyckes


Example:
The default Arduino platform configuration is:
configure_ADC(7,0,63); //Conversion time = 430 microseconds
Faster configuration is: (Average 8 samples)
configure_ADC(3,3,5); //Conversion time = 61 microseconds
Fasten configuration that works fine is:
configure_ADC(3,0,5); //Conversion time = 15 microseconds

The function returns:
	 0 no errors:
	-1 Prescaler out of range
	-2 SamplenNum out of range
	-3 Samplen out of range
*********************************************************************************/
int configure_ADC(unsigned long /*Prescaler*/,unsigned long /*SampleNum*/, unsigned char /*Samplen*/);
//
//#endif // ZEROCONFIGUREADC_H_INCLUDED