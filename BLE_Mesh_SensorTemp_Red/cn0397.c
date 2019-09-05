/**
******************************************************************************
*   @file     CN0397.c
*   @brief    Project main source file
*   @version  V0.1
*   @author   ADI
*
*******************************************************************************
* Copyright 2016(c) Analog Devices, Inc.
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*  - Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*  - Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in
*    the documentation and/or other materials provided with the
*    distribution.
*  - Neither the name of Analog Devices, Inc. nor the names of its
*    contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*  - The use of this software may or may not infringe the patent rights
*    of one or more patent holders.  This license does not release you
*    from the requirement that you obtain separate licenses from these
*    patent holders to use this software.
*  - Use of the software either in source or binary form, must be run
*    on or directly connected to an Analog Devices Inc. component.
*
* THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT, MERCHANTABILITY
* AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
* INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*******************************************************************************
**/
#include "CN0397.h"
#include "SPI_Comm.h"
#include "AD7798.h"
#include <math.h>
#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_gpio.h"
#include "wiced_rtos.h"
#include "wiced_platform.h"

extern uint8_t button_state = 1;

#define RED  	1
#define GREEN 	0
#define BLUE	2


uint8_t statusReg, idReg, ioReg, gainAdc;
uint16_t modeReg, configReg, offsetReg, fullscaleReg, dataReg;
uint8_t convFlag;

uint16_t adcValue[3];
float voltageValue[3], intensityValue[3], lightConcentration[3];


int barLine[3];


const uint8_t Channels[] = { 1, 0, 2};

static const char *colour[] = {
   [0] = "RED",
   [1] = "GREEN",
   [2] = "BLUE",
};

const uint8_t ColorPrint[] = { 31, 32, 34 };

const uint8_t Gain[8] = { 1, 2, 4, 8, 16, 32, 64, 128};

const float Lux_LSB[3] = {2.122, 2.124, 2.113};

const float Optimal_Levels[3] = {26909.0, 8880.0, 26909.0};


void CN0397_DisplayData(void)
{

	  uint8_t channel, i;

	  WICED_BT_TRACE("\n\r");

	   for(channel = 0; channel < CHANNELS; channel++){

	   WICED_BT_TRACE("\tColor %s \t\t", colour[channel]);
	   }
	   WICED_BT_TRACE("\n\r");

	   for(channel = 0; channel < CHANNELS; channel++){

	   WICED_BT_TRACE(" Light Intensity = %d lux \t", (uint32_t)intensityValue[channel]);
	   }

	   WICED_BT_TRACE("\n\r");

//
//	   for(channel = 0; channel < CHANNELS; channel++){
//
//		   WICED_BT_TRACE(" Light Concentration = %d \t", (uint32_t)(lightConcentration[channel]*1000));
//	   }
//	   WICED_BT_TRACE("\n\r");

}


void CN0397_ReadADCData(uint8_t adcChannel, uint16_t *adcData)
{

   uint8_t channel;

   channel = 0x80 | adcChannel;

   convFlag = 1;

   wiced_hal_gpio_set_pin_output(CS_1, GPIO_PIN_OUTPUT_LOW);

   AD7798_SetRegisterValue(AD7798_REG_MODE, 0x200A, 2);

   while((AD7798_GetRegisterValue( AD7798_REG_STAT,1) & channel) != channel);

   wiced_rtos_delay_milliseconds(150,ALLOW_THREAD_TO_SLEEP);

   *adcData = AD7798_GetRegisterValue(AD7798_REG_DATA,2);

   wiced_hal_gpio_set_pin_output(CS_1, GPIO_PIN_OUTPUT_HIGH);

   convFlag = 0;

}

void CN0397_ConvertToVoltage(uint16_t adcValue, float *voltage)
{

   *voltage = (float)(adcValue * V_REF)/(float)(_2_16 * gainAdc);

}

void CN0397_Init(void)
{
   uint8_t channel;

   AD7798_Reset();

   if(AD7798_Init()){

         AD7798_SetCodingMode(AD7798_UNIPOLAR);
         AD7798_SetMode(AD7798_MODE_SINGLE,AD7798__FS_123);	// Single conversion Update Rate 50Hz
         AD7798_SetGain(ADC_GAIN);
         AD7798_SetFilter(ADC_SPS);
         AD7798_SetReference(AD7798_REFDET_ENA);

   }

   WICED_BT_TRACE("Sensor Detected aand Initialized\r\n");
   gainAdc = Gain[ADC_GAIN];

#if(USE_CALIBRATION == YES)

   for(channel = 0; channel < CHANNELS; channel++){

	   CN0397_Calibration(Channels[channel]);
	   WICED_BT_TRACE("Channel %s is calibrated!\r\n",colour[channel] );

   }
   WICED_BT_TRACE("System calibration complete!\r\n");

#endif
}

void CN0397_CalcLightIntensity(uint8_t channel, uint16_t adcValue, float *intensity)
{

   *intensity = adcValue * Lux_LSB[channel];

}

void CN0397_CalcLightConcentration(uint8_t channel, float intensity, float *conc)
{

   *conc = (intensity *100)/Optimal_Levels[channel];

}

void CN0397_SetAppData(uint8_t channel)
{
    uint8_t rgbChannel;

   rgbChannel = Channels[channel];

   AD7798_SetChannel(channel);

   CN0397_ReadADCData(channel, &adcValue[rgbChannel]);
   CN0397_ConvertToVoltage(adcValue[rgbChannel], &voltageValue[rgbChannel]);
   CN0397_CalcLightIntensity(rgbChannel, adcValue[rgbChannel], &intensityValue[rgbChannel]);
//   CN0397_CalcLightConcentration(rgbChannel, intensityValue[rgbChannel], &lightConcentration[rgbChannel]);
//   CN0397_SetBar(lightConcentration[rgbChannel], &barLine[rgbChannel]);
 }

void CN0397_Calibration(uint8_t channel)
{

   uint16_t setValue;

   AD7798_SetChannel(channel);  //select channel to calibrate

   // Perform system zero-scale calibration
   setValue = AD7798_GetRegisterValue(AD7798_REG_MODE, 2);

   setValue &= ~(AD7798_MODE_SEL(0x07));
   setValue |= AD7798_MODE_SEL(AD7798_MODE_CAL_SYS_ZERO);
   AD7798_SetRegisterValue(AD7798_REG_MODE, setValue, 2);

   while((AD7798_GetRegisterValue( AD7798_REG_STAT,1) & channel) != channel);  // wait for RDY bit to go low
   while(AD7798_GetRegisterValue(AD7798_REG_MODE, 2) != 0x4005);    // wait for ADC to go in idle mode


}

void CN0397_SetBar(float conc, int *line)
{
   float concLimit = 5.0;
   int i = 0, j;
   *line = 0;

   if (conc > 0.0){
      i = 1;
      *line = i;
   }

   for(j = 0; j< 20; j++){
     if(conc >= concLimit){
            *line = i+1;
      }
      concLimit +=5.0;
      i +=1;
   }

}


void CN0397_ReadData(uint8_t color, uint16_t *red_sensor, uint16_t *green_sensor, uint16_t *blue_sensor)
{
	CN0397_SetAppData(color);

	switch (color)
	{
		case RED:
			*red_sensor = (uint16_t)intensityValue[GREEN];

			break;

		case GREEN:
			*green_sensor = (uint16_t)intensityValue[RED];
			 WICED_BT_TRACE("green_sensor = %d\r\n", *green_sensor );

			break;

		case BLUE:
			*blue_sensor = (uint16_t)intensityValue[BLUE];
			break;
	}

}




