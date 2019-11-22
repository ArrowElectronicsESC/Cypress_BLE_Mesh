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
#include "stdint.h"
#include "sparcommon.h"
#include "cn0216.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_gpio.h"
#include "wiced_hal_pspi.h"
#include "wiced_rtos.h"
#include "wiced_platform.h"
#include "SPI_Comm.h"
#include <math.h>


#define	CALIBRATE	0

/*SPI 1 defines*/
#define CLK_1                                 WICED_P09
#define MISO_1                                WICED_P17
#define MOSI_1                                WICED_P06
#define CS_1                                  WICED_P15

/* 1 MHz frequency*/
#define DEFAULT_FREQUENCY                     (1000000u)

/* SPI register configuration macro*/
#define GPIO_CFG(CS_1,CLK_1,MOSI_1,MISO_1)    ((((UINT32)CS_1&0xff)<<24)|((UINT32)CLK_1&0xff)<<16)|(((UINT32)MOSI_1&0xff)<<8)|((UINT32)MISO_1)

/* SPI Comm message buffer */
uint8_t	 rec_msg[3];

/* Calibaration Constants */
#define	GRAMS_PER_CODE 	-5060
#define ZERO_SCALE		840578970
#define FULL_SCALE		642095460

//  Weigh Scale Variables
float ui16calibrationWeight = 100;	  //value is in units (grams)

// AD7791 variables
uint32_t ui32Adcdata = 0;

// Main program variables
float fgramsPerCode = 0.0;
float fWeight = 0.0;
float fzeroScaleCalibration = 0.0;
float ffullScaleCalibration = 0.0;

uint32_t	read_data;

void Ad7791INIT(void)
{
    /* Init the SPI Hardware - MSB First and Mode 3 are required for the CN0397 */

    wiced_hal_pspi_init(SPI1,
                        SPI_MASTER,
                        INPUT_PIN_PULL_UP,
                        GPIO_CFG(CS_1,CLK_1,MOSI_1,MISO_1),
                        DEFAULT_FREQUENCY,
                        SPI_MSB_FIRST,
                        SPI_SS_ACTIVE_LOW,
                        SPI_MODE_3,
                        CS_1);

    wiced_rtos_delay_milliseconds(50,ALLOW_THREAD_TO_SLEEP);

	writeAd7791 (RESET, 0xFF);			//Resets the AD7791
	wiced_rtos_delay_milliseconds(500,ALLOW_THREAD_TO_SLEEP);

	read_data = readAd7791(MODE_READ);
	WICED_BT_TRACE(" Mode Register = %x  \r\n", read_data);

	writeAd7791 (MODE_WRITE, 0x00);	    //Mode register value (single conversion, +/- Vref input, unbuffered mode)
	read_data = readAd7791(FILTER_READ);
	WICED_BT_TRACE(" Filter Register = %x  \r\n", read_data);

	writeAd7791 (FILTER_WRITE, 0x07);	// Filter register value (clock not divided down, 9.5 Hz update rate)

	WICED_BT_TRACE(" Init Complete \r\n" );
    wiced_rtos_delay_milliseconds(500,ALLOW_THREAD_TO_SLEEP);
}

void Ad7791_Calibration(void)
{
#if	CALIBRATE == 1
    // calibrating the scale
	 WICED_BT_TRACE(" Zero Scale Calibration in process, please wait 15 seconds \r\n");
	 wiced_rtos_delay_milliseconds(1000,ALLOW_THREAD_TO_SLEEP);
     fzeroScaleCalibration = getCalibrationData();

	 wiced_rtos_delay_milliseconds(1000,ALLOW_THREAD_TO_SLEEP);
     WICED_BT_TRACE(" Zero Scale Calibration Complete \r\n");
     WICED_BT_TRACE(" Please put %d grams Calibration Weight on the Scale \r\n", (uint32_t)(ui16calibrationWeight));

     wiced_rtos_delay_milliseconds(8000,ALLOW_THREAD_TO_SLEEP);
     WICED_BT_TRACE(" Full Scale Calibration in process, please wait 15 seconds \r\n");

     ffullScaleCalibration = getCalibrationData();

     WICED_BT_TRACE(" Full Scale Calibration Complete \r\n");
     WICED_BT_TRACE(" Calibration Process Complete \r\n");

     wiced_rtos_delay_milliseconds(3000,ALLOW_THREAD_TO_SLEEP);
#endif

#if !CALIBRATE
    	fgramsPerCode = GRAMS_PER_CODE/ 100000000.0;
    	fzeroScaleCalibration = ZERO_SCALE / 100.0;
    	ffullScaleCalibration = FULL_SCALE / 100.0;
#endif
}
uint32_t readAd7791 (uint8_t ui8address)
{
	uint8_t ui8AdcUpperCodes;			// Data register bits 23-16
	uint8_t ui8AdcMiddleCodes;			// Data register bits 15-8
	uint8_t ui8AdcLowerCodes;			// Data register bits 7-0
	uint32_t ui32AdcCodes = 0;
	uint8_t	 rec_msg[3];

	ui8AdcUpperCodes = 0;			// Data register bits 23-16
	ui8AdcMiddleCodes = 0;			// Data register bits 15-8
	ui8AdcLowerCodes = 0;			// Data register bits 7-0
	ui32AdcCodes = 0;

	rec_msg[0] = ui8address;
//	WICED_BT_TRACE(" ADC Register Being Read = %x  ", ui8address);

    wiced_hal_gpio_set_pin_output(CS_1, GPIO_PIN_OUTPUT_LOW);
	if (ui8address == DATA_READ)
	{
	    wiced_hal_pspi_tx_data(SPI1, 1, (uint8_t*)rec_msg);

	     /* Receiving response from slave*/
	    wiced_hal_pspi_rx_data(SPI1, 3, (uint8_t*)rec_msg);

		ui8AdcUpperCodes = rec_msg[0];			// Data register bits 23-16
		ui8AdcMiddleCodes =rec_msg[1];			// Data register bits 15-8
		ui8AdcLowerCodes = rec_msg[2];			// Data register bits 7-0
		ui32AdcCodes = ((long)ui8AdcUpperCodes << 16) | ((long)ui8AdcMiddleCodes << 8) | ui8AdcLowerCodes;

//		WICED_BT_TRACE(" ADC Register Data = %x %x %X  \r\n", rec_msg[0], rec_msg[1], rec_msg[2] );
	}
	else
	{

	    wiced_hal_pspi_tx_data(SPI1, 1, (uint8_t*)rec_msg);

	     /* Receiving response from slave*/
	    wiced_hal_pspi_rx_data(SPI1, 1, (uint8_t*)rec_msg);

		ui8AdcLowerCodes = rec_msg[0];			// register read

		ui32AdcCodes = ((long)ui8AdcUpperCodes << 16) | ((long)ui8AdcMiddleCodes << 8) | ui8AdcLowerCodes;
//		WICED_BT_TRACE(" ADC Register Data = %x %x %x \r\n", rec_msg[0], rec_msg[1],  rec_msg[2] );
	}
    wiced_hal_gpio_set_pin_output(CS_1, GPIO_PIN_OUTPUT_HIGH);

	return ui32AdcCodes;
}

void writeAd7791 (uint8_t ui8address, uint8_t ui8value)
{
	uint8_t reset_command[4] = {0xff, 0xff, 0xff, 0xff};
    wiced_hal_gpio_set_pin_output(CS_1, GPIO_PIN_OUTPUT_LOW);
	if (ui8address != RESET)
	{
		rec_msg[0] = ui8address;
		rec_msg[1] = ui8value;
	    wiced_hal_pspi_tx_data(SPI1, 2, (uint8_t*)rec_msg);
	}

	else
	{
		spi_sensor_write(4, &reset_command);

		WICED_BT_TRACE(" Reset Command \r\n");			//Debug serial prints
	}

    wiced_hal_gpio_set_pin_output(CS_1, GPIO_PIN_OUTPUT_HIGH);
//	WICED_BT_TRACE(" ADC Register Write = %x  Data = %x \r\n", ui8address, ui8value);


}

float getWeight(void)
{

	do
	{
        	ui32Adcdata = readAd7791(STATUS_READ);

   	}while (ui32Adcdata & 0x80);

        ui32Adcdata = readAd7791(DATA_READ);

       fgramsPerCode = ui16calibrationWeight / (ffullScaleCalibration - fzeroScaleCalibration);


        fWeight = ((float)ui32Adcdata - fzeroScaleCalibration) * fgramsPerCode;
        return (fWeight);
}
float getCalibrationData(void)
{
    uint32_t ui32calibrationData = 0;
    uint32_t ui32status = 0;
    uint8_t x = 0;
    float fcalibration = 0.0;

    for (x = 0; x < 100; x++)
    {
        ui32status = 0;
        do
        {
          ui32status = readAd7791(STATUS_READ);

        }while (ui32status & 0x80);

        ui32calibrationData += readAd7791(DATA_READ);
        wiced_rtos_delay_milliseconds(100,ALLOW_THREAD_TO_SLEEP);
    }

    fcalibration = (float)(ui32calibrationData)/100.0;
	return fcalibration;
}




