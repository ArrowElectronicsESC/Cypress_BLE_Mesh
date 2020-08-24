/*
 * Copyright 2019, Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All Rights Reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software"), is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/** File name: spi_master_w_sensor.c
 *
 * WICED sample application for SPI master running SPI instance for am  the
 * ADI CNB216 Precision Weight Scale Design Arduimo Shield
 *
 * This application demonstrates how to use SPI driver interface to connect to an
 * Arduino shield with a SPI sensor
 *
 * Features demonstrated:
 * - SPI WICED APIs
 * - SPI Drivers for the ADI CN0397 Light Sensor Evaul Board
 * - WICED RTOS APIs
 *
 * Hardware Connections:
 *
 * SPI sensor   Platform Name	Arduino Header Name
 * CLK     		WICED_P09			D13
 * MISO    		WICED_P17			D12
 * MOSI    		WICED_P06			D11
 * CS      		WICED_P15			D10
 * GND
 *
 */

/******************************************************************************
 *                                Includes
 ******************************************************************************/

#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_gpio.h"
#include "wiced_platform.h"
#include "wiced_hal_pspi.h"
#include "wiced_hal_puart.h"
#include "wiced_rtos.h"
#include "wiced_bt_stack.h"
#include "wiced_rtc.h"
#include "wiced_hal_sflash.h"
#include "wiced_hal_nvram.h"
#include "cn0216.h"


/******************************************************************************
 *                                Constants
 ******************************************************************************/
/* Threads defines */
/* Sensible stack size for most threads*/
#define THREAD_STACK_MIN_SIZE                 (1024)
/* Defining thread priority levels*/
#define PRIORITY_HIGH                         (3)
#define PRIORITY_MEDIUM                       (5)
#define PRIORITY_LOW                          (7)

///*SPI 1 defines*/
//
//#define CLK_1                                 WICED_P09
//#define MISO_1                                WICED_P17
//#define MOSI_1                                WICED_P06
//#define CS_1                                  WICED_P15
//
///* 1 MHz frequency*/
//#define DEFAULT_FREQUENCY                     (1000000u)
//
///* SPI register configuration macro*/
//#define GPIO_CFG(CS_1,CLK_1,MOSI_1,MISO_1)    ((((UINT32)CS_1&0xff)<<24)|((UINT32)CLK_1&0xff)<<16)|(((UINT32)MOSI_1&0xff)<<8)|((UINT32)MISO_1)

#define SLEEP_TIMEOUT                         (1000)

/* Delay between transmitting and receiving SPI messages from sensor, to prevent reading earlier responses.*/
#define TX_RX_TIMEOUT                         (50)

//  Weigh Scale Variables
uint16_t ui16calibrationWeight = 1000;	  //value is in units (grams)

// AD7791 variables
uint32_t ui32Adcdata = 0;

// Main program variables
float fgramsPerCode = 0.0;
float fWeight = 0.0;
float fzeroScaleCalibration = 0.0;
float ffullScaleCalibration = 0.0;


/******************************************************************************
 *                                Structures
 ******************************************************************************/

/* pSPI data packet*/
typedef struct
{
    uint16_t data ;
    uint16_t header;
}data_packet;

/*Temperature record structure stored in sflash*/
typedef struct
{
    uint16_t record_no;
    uint8_t dec_temp;
    uint8_t frac_temp;
    wiced_rtc_time_t timestamp;
}temperature_record;

/* Enumeration listing SPI sensor commands
 * GET_MANUFACTURER_ID: Command to get Manufacturer ID.
 * GET_UNIT: Command to get unit scale.
 * MEASURE_TEMPERATURE: Command to get temperature reading.*/
typedef enum
{
    GET_MANUFACTURER_ID = 1,
    GET_UNIT,
    MEASURE_TEMPERATURE
}sensor_cmd;

/* Enumeration listing SPI Master states
 * SENSOR_DETECT: Master checks for presence of SLAVE by verifying received
 *                packet header and obtains response to Manufacturer ID on
 *                verification.
 * GET_UNIT: Master checks for presence of SLAVE by verifying received packet
 *           header and obtains response to Unit ID on verification.
 * GET_TEMPERATURE: Master checks for presence of SLAVE by verifying received
 *                  packet header and obtains response to Temperature reading on
 *                  verification.*/
typedef enum
{
    SENSOR_INIT,
    READ_AD7791,
    }master_state;

/******************************************************************************
 *                                Variables Definitions
 ******************************************************************************/

static wiced_thread_t       *spi_1;

static wiced_semaphore_t    *sem;

static uint8_t button_state;

/******************************************************************************
 *                                Function Prototypes
 ******************************************************************************/

wiced_result_t bt_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
void           initialize_app( void );
static void    spi_sensor_thread( uint32_t arg);


/**************************************************************************************
 *                                Function Definitions
 *************************************************************************************/
/*******************************************************************************
* Function Name: void application_start(void)
********************************************************************************
* Summary: Initialize transport configuration and register BLE
*          management event callback.
*
* Parameters:
*   None
*
* Return:
*   None
*
********************************************************************************/

void application_start(void)
{
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );
    if(WICED_SUCCESS != wiced_bt_stack_init( bt_cback, NULL, NULL ))
    {
        WICED_BT_TRACE("BT stack initialization failed \n\r");
    }
}

/**********************************************************************************************
* Function Name: wiced_result_t bt_cback(wiced_bt_management_evt_t event,
*                                        wiced_bt_management_evt_data_t *p_event_data)
***********************************************************************************************
* Summary:
*   This is a Bluetooth management event handler function to receive events from
*   BLE stack and process as per the application.
*
* Parameters:
*   wiced_bt_management_evt_t event             : BLE event code of one byte length
*   wiced_bt_management_evt_data_t *p_event_data: Pointer to BLE management event structures
*
* Return:
*  wiced_result_t: Error code from WICED_RESULT_LIST or BT_RESULT_LIST
*
***********************************************************************************************/

wiced_result_t bt_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_result_t result = WICED_SUCCESS;

    switch( event )
    {
    /* BlueTooth stack enabled*/
    case BTM_ENABLED_EVT:
    	initialize_app();
        break;

    default:
        break;
    }
    return result;
}

/*******************************************************************************
* Function Name: void initialize_app( void )
********************************************************************************
* Summary:This functions initializes the SPI,SFLASH, threads, message queue,
*         semaphore and GPIO
*
* Parameters:
*   None
*
* Return:
*   None
*
*******************************************************************************/

void initialize_app( void )
{
    wiced_result_t  result;
    wiced_result_t  status;
    uint16_t        record = 0;


    WICED_BT_TRACE("\r\n CN0216 ADI Weight Sensor Demo\r\n");

    wiced_hal_gpio_configure_pin( WICED_GPIO_PIN_BUTTON_1,
                                  GPIO_PULL_UP,
                                  GPIO_PIN_OUTPUT_HIGH);


 //   button_state =  wiced_hal_gpio_get_pin_input_status(WICED_GPIO_PIN_BUTTON_1);


    /* Initialize RTC. RTC by default is set to the time 00:00:00 Hrs, January 1, 2010.*/
    wiced_rtc_init();

    wiced_rtos_delay_milliseconds(50,ALLOW_THREAD_TO_SLEEP);

    spi_1 = wiced_rtos_create_thread();
    if ( WICED_SUCCESS == wiced_rtos_init_thread(spi_1,
                                                 PRIORITY_MEDIUM,
                                                 "SPI 1 instance",
                                                 spi_sensor_thread,
                                                 THREAD_STACK_MIN_SIZE,
                                                 NULL ) )
    {
        WICED_BT_TRACE( " SPI Sensor thread created\n\r" );
    }
    else
    {
        WICED_BT_TRACE( "Failed to create SPI Sensor thread \n\r" );
    }
 }

/*******************************************************************************
* Function Name: void spi_sensor_thread(uint32_t arg )
********************************************************************************
* Summary:Starts and maintains transfer of SPI sensor data.
*
* Parameters:
*   uint32_t arg: unused argument
*
* Return:
*   None
*
*******************************************************************************/

void spi_sensor_thread(uint32_t arg )
{
    wiced_result_t result;
    uint32_t count;
    temperature_record sampled_temp;
    master_state curr_state = SENSOR_INIT;
    uint32_t	loop_count;
    uint8_t calibrated = 0;

    while(WICED_TRUE)
    {
        switch(curr_state)
        {
        case SENSOR_INIT:

            /* Init CN0397 and read ID */
        	Ad7791INIT();
        	loop_count = 0;
        	wiced_rtos_delay_milliseconds(100,ALLOW_THREAD_TO_SLEEP);
        	if (!calibrated)
        	{
        		Ad7791_Calibration();
        		calibrated = 1;

        	}

        	curr_state = READ_AD7791;

            break;

        case READ_AD7791:

        	fWeight = getWeight();

        	WICED_BT_TRACE(" Weight = %d grams \r\n", (uint32_t) fWeight);

        	wiced_rtos_delay_milliseconds(500,ALLOW_THREAD_TO_SLEEP);
        	loop_count++;
        	if (loop_count> 511)
        	{
        		curr_state = SENSOR_INIT;
        	}

            break;

        default:

        	break;
        }
    }
 }

