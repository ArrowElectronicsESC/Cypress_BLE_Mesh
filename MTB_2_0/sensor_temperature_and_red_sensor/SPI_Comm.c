/**
******************************************************************************
*   @file     Communication.c
*   @brief    Source file for communication part.
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

/***************************** Include Files **********************************/
#include "SPI_Comm.h"
#include "wiced_hal_gpio.h"
#include "wiced_hal_pspi.h"
#include "wiced_rtos.h"
#include "wiced_bt_trace.h"

/*******************************************************************************
* Function Name: void spi_sensor_write(uint8_t byteCount, uint8_t *send_msg)
********************************************************************************
* Summary:Utility function that performs SPI Write with SPI sensor.
*
*
* @param data - Write data buffer:
*               - first byte is the chip select number;
*               - from the second byte onwards are located data bytes to write.
* @param bytesNumber - Number of bytes to write.
*
*
* Return:
*   None
*
*******************************************************************************/

void  spi_sensor_write(uint8_t byteCount, uint8_t *send_msg)
{
    /* Chip select is set to LOW to select the slave for SPI transactions*/
    wiced_hal_gpio_set_pin_output(CS_1, GPIO_PIN_OUTPUT_LOW);

    /* Writ data to slave */
    wiced_hal_pspi_tx_data(SPI1,
                           byteCount,
                           (uint8_t*)send_msg);

    /* Chip select is set to HIGH when SPI transaction is complete*/
    wiced_hal_gpio_set_pin_output(CS_1, GPIO_PIN_OUTPUT_HIGH);
    return;
}

/*******************************************************************************
* Function Name: void spi_sensor_read(uint8_t byteCount, uint8_t *rec_msg)
********************************************************************************
 * @brief Reads data from SPI.
 *
 * @param data - As an input parameter, data represents the write buffer:
 *               - first byte is commnad;
 *               - from the second byte onwards are located data bytes to read.
 *               As an output parameter, data represents the read buffer:
 *               - from the first byte onwards are located the read data bytes.
 *
 * @param bytesNumber - Number of bytes to write.
 *
 * @return None
*******************************************************************************/

void  spi_sensor_read(uint8_t byteCount, uint8_t *rec_msg)
{


    /* Chip select is set to LOW to select the slave for SPI transactions*/
    wiced_hal_gpio_set_pin_output(CS_1, GPIO_PIN_OUTPUT_LOW);

    /* Write Communication Register */
    wiced_hal_pspi_tx_data(SPI1,
                           1,
                           (uint8_t*)rec_msg);

     /* Receiving response from slave*/
    wiced_hal_pspi_rx_data(SPI1,
    					   byteCount,
						   (uint8_t*)rec_msg);

    /* Chip select is set to HIGH when SPI transaction is complete*/
    wiced_hal_gpio_set_pin_output(CS_1, GPIO_PIN_OUTPUT_HIGH);
    return;
}
