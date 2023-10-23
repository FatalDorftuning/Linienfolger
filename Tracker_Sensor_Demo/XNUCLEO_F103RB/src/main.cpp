/*********************************************************************************************************
*
* File                : main.cpp
* Hardware Environment: 
* Build Environment   : Keil MDK
* Version             : V4.74
* By                  : WaveShare
*
*                                  (c) Copyright 2005-2014, WaveShare
*                                       http://www.waveshare.net
*                                       http://www.waveshare.com
*                                          All Rights Reserved
*
*********************************************************************************************************/
#include "mbed.h"


/*
variable
*/
AnalogIn IR1(A0);
AnalogIn IR2(A1);
AnalogIn IR3(A2);
AnalogIn IR4(A3);
AnalogIn IR5(A4);

int main(void) 
{           
    while(1) 
    {
			int temp[5];
			temp[0] = IR1.read_u16()/64;
			temp[1] = IR2.read_u16()/64;
			temp[2] = IR3.read_u16()/64;
			temp[3] = IR4.read_u16()/64;
			temp[4] = IR5.read_u16()/64;
			printf("%4d  %4d  %4d  %4d  %4d \r\n",temp[0],temp[1],temp[2],temp[3],temp[4]);  
			wait(0.1);
    }
}




