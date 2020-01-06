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

/** @file
 *
 * This demo application shows a implementation of a temperature sensor and
 * read the red light sensor on the Analog Devices CNO397 RGB light Sensor Arduino Shield.
 * The app is based on the snip/mesh/mesh_sensor_server sample which
 * implements BLE Mesh Sensor Server model.
 *
 * Features demonstrated
 *  - Temperature measurement using the on board Thermistor on the EVK
 *  - Adding a 2nd sensor element, the Red Light Sensor on the Arduino Shield
 *
 * To demonstrate the app, walk through the following steps.
 * 1. Build and download the application (to the WICED board)
 * 2. Use Android or iOS MeshController app  and provision the temperature sensor
 * 3. After successful provisioning, user can use the Android  or iOS MeshController/Mesh Client
 *    to configure the below parameters of the sensor
 *
 *    a> configure sensor to publish the sensor data to a specific group or to all-nodes.
 *    b> configure publish period : publish period defines how often the user wants the sensor to publish the data.
 *    c> set cadence of the sensor :
 *       set minimum interval in which sensor data has to be published.
 *       set the range in which the fast cadence has to be observed.
 *       set the fast cadence period (how fast the data has to be published with respect to publish period).
 *       set the unit in which if the values change the data should be published and trigger type (Native or percentage).
 *           example : publish data if the data changes by 2 units/10%
 * 4. To change the temperature on the thermistor, you can keep your finger on the sensor and see the changes.
 *
 * This is a test
 *
 */
#include "wiced_bt_uuid.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_mesh_models.h"
#include "wiced_bt_trace.h"
#include "wiced_timer.h"
#include "wiced_bt_mesh_app.h"
#include "wiced_thermistor.h"
#include "wiced_hal_nvram.h"
#include "wiced_sleep.h"
#include "cn0397.h"
#include "wiced_platform.h"
#include "wiced_hal_gpio.h"
#include "wiced_hal_pspi.h"
#include "wiced_RTOS.h"
#include "wiced_rtc.h"

#include "wiced_bt_cfg.h"
extern wiced_bt_cfg_settings_t wiced_bt_cfg_settings;

/******************************************************
 *          Constants
 ******************************************************/
#define MESH_PID                0x3122
#define MESH_VID                0x0002
#define MESH_FWID               0x3122000101010001
#define MESH_CACHE_REPLAY_SIZE  0x0008

#define MESH_TEMP_SENSOR_PROPERTY_ID                    WICED_BT_MESH_PROPERTY_PRESENT_AMBIENT_TEMPERATURE
#define MESH_TEMP_SENSOR_VALUE_LEN                      WICED_BT_MESH_PROPERTY_LEN_PRESENT_AMBIENT_TEMPERATURE

// The onboard thermistor hardware has a positive and negative tolerance of 1%
#define MESH_TEMPERATURE_SENSOR_POSITIVE_TOLERANCE      CONVERT_TOLERANCE_PERCENTAGE_TO_MESH(1)
#define MESH_TEMPERATURE_SENSOR_NEGATIVE_TOLERANCE      CONVERT_TOLERANCE_PERCENTAGE_TO_MESH(1)

#define MESH_TEMPERATURE_SENSOR_SAMPLING_FUNCTION       WICED_BT_MESH_SENSOR_SAMPLING_FUNCTION_UNKNOWN
#define MESH_TEMPERATURE_SENSOR_MEASUREMENT_PERIOD      WICED_BT_MESH_SENSOR_VAL_UNKNOWN
#define MESH_TEMPERATURE_SENSOR_UPDATE_INTERVAL         WICED_BT_MESH_SENSOR_VAL_UNKNOWN

#define MESH_LIGHT_SENSOR_PROPERTY_ID                  	WICED_BT_MESH_PROPERTY_PRESENT_AMBIENT_LIGHT_LEVEL
#define MESH_LIGHT_SENSOR_VALUE_LEN                   	WICED_BT_MESH_PROPERTY_LEN_PRESENT_AMBIENT_LIGHT_LEVE

#define	MESH_LIGHT_SENSOR_POSITIVE_TOLERANCE            WICED_BT_MESH_SENSOR_TOLERANCE_UNSPECIFIED
#define MESH_LIGHT_NEGATIVE_TOLERANCE           		WICED_BT_MESH_SENSOR_TOLERANCE_UNSPECIFIED

#define MESH_LIGHT_SENSOR_SAMPLING_FUNCTION             WICED_BT_MESH_SENSOR_SAMPLING_FUNCTION_UNKNOWN
#define MESH_LIGHT_SENSOR_MEASUREMENT_PERIOD            WICED_BT_MESH_SENSOR_VAL_UNKNOWN
#define MESH_LIGHT_SENSOR_UPDATE_INTERVAL               WICED_BT_MESH_SENSOR_VAL_UNKNOWN

#define MESH_TEMPERATURE_SENSOR_CADENCE_NVRAM_ID        WICED_NVRAM_VSID_START

// The Red Sensor defines
#define MESH_RED_SENSOR_POSITIVE_TOLERANCE        	   	WICED_BT_MESH_SENSOR_TOLERANCE_UNSPECIFIED
#define MESH_RED_SENSOR_NEGATIVE_TOLERANCE          	WICED_BT_MESH_SENSOR_TOLERANCE_UNSPECIFIED

#define MESH_RED_SENSOR_SAMPLING_FUNCTION            	WICED_BT_MESH_SENSOR_SAMPLING_FUNCTION_UNKNOWN
#define MESH_RED_SENSOR_MEASUREMENT_PERIOD           	WICED_BT_MESH_SENSOR_VAL_UNKNOWN
#define MESH_RED_SENSOR_UPDATE_INTERVAL              	WICED_BT_MESH_SENSOR_VAL_UNKNOWN

#define MESH_RED_SENSOR_CADENCE_VSID	          		WICED_NVRAM_VSID_START + 0x100

/*SPI 1 defines to interface to ADI CN0397 Arduino Shield */
#define CLK_1                                 WICED_P09
#define MISO_1                                WICED_P17
#define MOSI_1                                WICED_P06
#define CS_1                                  WICED_P15

/* 1 MHz frequency*/
#define DEFAULT_FREQUENCY                     (1000000u)

/* SPI register configuration macro*/
#define GPIO_CFG(CS_1,CLK_1,MOSI_1,MISO_1)    ((((UINT32)CS_1&0xff)<<24)|((UINT32)CLK_1&0xff)<<16)|(((UINT32)MOSI_1&0xff)<<8)|((UINT32)MISO_1)

/* Color Sensor Channel Defines */
#define RED  	1
#define GREEN 	0
#define BLUE	2


/******************************************************
 *          Structures
 ******************************************************/

/******************************************************
 *          Function Prototypes
 ******************************************************/
static void         mesh_app_init(wiced_bool_t is_provisioned);
static wiced_bool_t mesh_app_notify_period_set(uint8_t element_idx, uint16_t company_id, uint16_t model_id, uint32_t period);
static void         mesh_app_lpn_sleep(uint32_t timeout);
static void         mesh_app_factory_reset(void);
static void         mesh_sensor_server_restart_timer(wiced_bt_mesh_core_config_sensor_t *p_sensor);
static void         mesh_temp_sensor_server_report_handler(uint16_t event, uint8_t element_idx, void *p_get_data, void *p_ref_data);
static void         mesh_red_sensor_server_report_handler(uint16_t event, uint8_t element_idx, void *p_get_data, void *p_ref_data);
static void         mesh_sensor_server_config_change_handler(uint8_t element_idx, uint16_t event, uint16_t property_id, uint16_t setting_property_id);
static void         mesh_sensor_server_send_status(wiced_bt_mesh_event_t *p_event, uint16_t property_id);
static void         mesh_sensor_server_process_cadence_changed(uint8_t element_idx, uint16_t property_id);
static void         mesh_sensor_server_process_setting_changed(uint8_t element_idx, uint16_t property_id, uint16_t setting_property_id);
static int8_t       mesh_sensor_get_temperature_8(void);

/* publish timer for Temperature sensor */
static void         mesh_sensor_publish_timer_callback(TIMER_PARAM_TYPE arg);

/* publish timer for Red Light sensor */
static void 		mesh_red_sensor_publish_timer_callback(TIMER_PARAM_TYPE arg);

static void         mesh_sensor_server_enter_hid_off(uint32_t timeout_ms);

#ifdef HCI_CONTROL
static void         mesh_sensor_hci_event_send_cadence_set(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_sensor_cadence_set_data_t *p_set);
static void         mesh_sensor_hci_event_send_setting_set(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_sensor_setting_set_data_t *p_set);
#endif

/******************************************************
 *          Variables Definitions
 ******************************************************/
uint8_t mesh_mfr_name[WICED_BT_MESH_PROPERTY_LEN_DEVICE_MANUFACTURER_NAME]          = { 'C', 'y', 'p', 'r', 'e', 's', 's', 0 };
uint8_t mesh_model_num[WICED_BT_MESH_PROPERTY_LEN_DEVICE_MODEL_NUMBER]              = { '1', '2', '3', '4', 0, 0, 0, 0 };
uint8_t mesh_prop_fw_version[WICED_BT_MESH_PROPERTY_LEN_DEVICE_FIRMWARE_REVISION] =   { '0', '6', '.', '0', '2', '.', '0', '5' }; // this is overwritten during init
uint8_t mesh_system_id[8]                                                           = { 0xbb, 0xb8, 0xa1, 0x80, 0x5f, 0x9f, 0x91, 0x71 };

// Present Ambient Temperature property uses Temperature 8 format, i.e. 0.5 degree Celsius.
int8_t        mesh_sensor_current_temperature = 42; // 21 degree Celsius
int8_t        mesh_sensor_sent_value = 0;           //
uint32_t      mesh_sensor_sent_time;                // time stamp when temperature was published
uint32_t      mesh_sensor_publish_period = 0;       // publish period in msec
uint32_t      mesh_sensor_fast_publish_period = 0;  // publish period in msec when values are outside of limit
uint32_t      mesh_sensor_sleep_timeout = 0;        // timeout value in msec that is currently running
wiced_timer_t mesh_sensor_cadence_timer;

/* Present Red Sensor property */
uint32_t      mesh_light_sensor_current_value;
uint8_t       mesh_red_sensor_current_value[3];
uint8_t       mesh_red_sensor_sent_value[3] = {0,0,0};  // Ambient Light Sensor Element required 3 octets
uint32_t      mesh_red_sensor_sent_time;                // time stamp when temperature was published
uint32_t      mesh_red_sensor_publish_period = 0;       // publish period in msec
uint32_t      mesh_red_sensor_fast_publish_period = 0;  // publish period in msec when values are outside of limit
uint32_t      mesh_red_sensor_sleep_timeout = 0;        // timeout value in msec that is currently running
wiced_timer_t mesh_light_sensor_cadence_timer;

// Optional setting for the temperature sensor, the Total Device Runtime, in Time Hour 24 format
uint8_t mesh_temperature_sensor_setting0_val[] = { 0x01, 0x00, 0x00 };

uint16_t      	mesh_sensor_red_sensor = 0; 		// lux
uint16_t        mesh_sensor_green_sensor = 0; 		// lux
uint16_t        mesh_sensor_blue_sensor = 0; 		// lux

//uint8_t		button_state =1;

wiced_bt_mesh_core_config_model_t mesh_element1_models[] =
{
    WICED_BT_MESH_DEVICE,
    WICED_BT_MESH_MODEL_SENSOR_SERVER,
};
#define MESH_APP_NUM_MODELS  (sizeof(mesh_element1_models) / sizeof(wiced_bt_mesh_core_config_model_t))

wiced_bt_mesh_sensor_config_setting_t sensor_settings[] =
{
    {
        .setting_property_id = WICED_BT_MESH_PROPERTY_TOTAL_DEVICE_RUNTIME,
        .access              = WICED_BT_MESH_SENSOR_SETTING_READABLE_AND_WRITABLE,
        .value_len           = WICED_BT_MESH_PROPERTY_LEN_TOTAL_DEVICE_RUNTIME,
        .val                 = mesh_temperature_sensor_setting0_val
    },
};

#define MESH_APP_NUM_PROPERTIES (sizeof(mesh_element1_properties) / sizeof(wiced_bt_mesh_core_config_property_t))

wiced_bt_mesh_core_config_sensor_t mesh_element1_sensors[] =
{
    {
        .property_id = WICED_BT_MESH_PROPERTY_PRESENT_AMBIENT_TEMPERATURE,
        .prop_value_len = WICED_BT_MESH_PROPERTY_LEN_PRESENT_AMBIENT_TEMPERATURE,
        .descriptor =
        {
            .positive_tolerance = MESH_TEMPERATURE_SENSOR_POSITIVE_TOLERANCE,
            .negative_tolerance = MESH_TEMPERATURE_SENSOR_NEGATIVE_TOLERANCE,
            .sampling_function  = MESH_TEMPERATURE_SENSOR_SAMPLING_FUNCTION,
            .measurement_period = MESH_TEMPERATURE_SENSOR_MEASUREMENT_PERIOD,
            .update_interval    = MESH_TEMPERATURE_SENSOR_UPDATE_INTERVAL,
        },
        .data = (uint8_t*)&mesh_sensor_sent_value,
        .cadence =
        {
            // Value 1 indicates that cadence does not change depending on the measurements
            .fast_cadence_period_divisor = 1,
            .trigger_type_percentage     = WICED_FALSE,
            .trigger_delta_down          = 0,
            .trigger_delta_up            = 0,
            .min_interval                = (1 << 12), // minimum interval for sending data by default is 4 seconds
            .fast_cadence_low            = 0,
            .fast_cadence_high           = 0,
        },
        .num_series     = 0,
        .series_columns = NULL,
        .num_settings   = 1,
        .settings       = sensor_settings,
    },
};


wiced_bt_mesh_core_config_sensor_t mesh_element2_sensors[] =
{
    {
        .property_id    = WICED_BT_MESH_PROPERTY_PRESENT_AMBIENT_LIGHT_LEVEL,
        .prop_value_len = WICED_BT_MESH_PROPERTY_LEN_PRESENT_AMBIENT_LIGHT_LEVEL ,
        .descriptor =
        {
            .positive_tolerance = MESH_RED_SENSOR_POSITIVE_TOLERANCE,
            .negative_tolerance = MESH_RED_SENSOR_NEGATIVE_TOLERANCE,
            .sampling_function  = MESH_RED_SENSOR_SAMPLING_FUNCTION,
            .measurement_period = MESH_RED_SENSOR_MEASUREMENT_PERIOD,
            .update_interval    = MESH_RED_SENSOR_UPDATE_INTERVAL,
        },
        .data = (uint8_t*)&mesh_red_sensor_sent_value,
        .cadence =
        {
            // Value 0 indicates that cadence does not change depending on the measurements
            .fast_cadence_period_divisor = 32,          // Recommended publish period is 320sec, 32 will make fast period 10sec
            .trigger_type_percentage     = WICED_FALSE, // The Property is Bool, does not make sense to use percentage
            .trigger_delta_down          = 0,           // This will not cause message when presence changes from 1 to 0
            .trigger_delta_up            = 1,           // This will cause immediate message when presence changes from 0 to 1
            .min_interval                = (1 << 10),   // Milliseconds. Conversion to SPEC values is done by the mesh models library
            .fast_cadence_low            = 1,           // If fast_cadence_low is greater than fast_cadence_high and the measured value is either is lower
                                                        // than fast_cadence_high or higher than fast_cadence_low, then the message shall be published
                                                        // with publish period (equals to mesh_sensor_publish_period divided by fast_cadence_divisor_period)
            .fast_cadence_high           = 0,           // is more or equal cadence_low or less then cadence_high. This is what we need.
        },
        .num_series     = 0,
        .series_columns = NULL,
        .num_settings   = 0,
        .settings       = NULL,
    },
};

#define MESH_SENSOR_SERVER_ELEMENT_INDEX    0
#define MESH_TEMPERATURE_SENSOR_INDEX       0
#define MESH_RED_SENSOR_INDEX      			1

wiced_bt_mesh_core_config_model_t   mesh_element2_models[] =
{
		WICED_BT_MESH_MODEL_SENSOR_SERVER,
};
#define MESH_APP_NUM_MODELS_RED  (sizeof(mesh_element2_models) / sizeof(wiced_bt_mesh_core_config_model_t))

/* for future additions of green and blue sensors */

//wiced_bt_mesh_core_config_model_t   mesh_element3_models[] =
//{
//		WICED_BT_MESH_MODEL_SENSOR_SERVER,
//};

//#define MESH_APP_NUM_MODELS_GREEN  (sizeof(mesh_element3_models) / sizeof(wiced_bt_mesh_core_config_model_t))
//
//wiced_bt_mesh_core_config_model_t   mesh_element3_models[] =
//{
//		WICED_BT_MESH_MODEL_SENSOR_SERVER,
//};
//#define MESH_APP_NUM_MODELS_BLUE  (sizeof(mesh_element4_models) / sizeof(wiced_bt_mesh_core_config_model_t))
//

#define MESH_LIGHT_SENSOR_SERVER_ELEMENT_INDEX_RED     1
//#define MESH_LIGHT_SENSOR_SERVER_ELEMENT_INDEX_GREEN   2
//#define MESH_LIGHT_SENSOR_SERVER_ELEMENT_INDEX_BLUE    3

wiced_bt_mesh_core_config_element_t mesh_elements[] =
{
    {
        .location = MESH_ELEM_LOC_MAIN,                                  // location description as defined in the GATT Bluetooth Namespace Descriptors section of the Bluetooth SIG Assigned Numbers
        .default_transition_time = MESH_DEFAULT_TRANSITION_TIME_IN_MS,   // Default transition time for models of the element in milliseconds
        .onpowerup_state = WICED_BT_MESH_ON_POWER_UP_STATE_RESTORE,      // Default element behavior on power up
        .default_level = 0,                                              // Default value of the variable controlled on this element (for example power, lightness, temperature, hue...)
        .range_min = 1,                                                  // Minimum value of the variable controlled on this element (for example power, lightness, temperature, hue...)
        .range_max = 0xffff,                                             // Maximum value of the variable controlled on this element (for example power, lightness, temperature, hue...)
        .move_rollover = 0,                                              // If true when level gets to range_max during move operation, it switches to min, otherwise move stops.
        .properties_num = 0,                                             // Number of properties in the array models
        .properties = NULL,                                              // Array of properties in the element.
        .sensors_num = 1,                                                // Number of properties in the array models
        .sensors = mesh_element1_sensors,                                // Array of properties in the element.
        .models_num = MESH_APP_NUM_MODELS,                               // Number of models in the array models
        .models = mesh_element1_models,                                  // Array of models located in that element. Model data is defined by structure wiced_bt_mesh_core_config_model_t
    },
    {
        .location = MESH_ELEM_LOC_MAIN,                                  // location description as defined in the GATT Bluetooth Namespace Descriptors section of the Bluetooth SIG Assigned Numbers
        .default_transition_time = MESH_DEFAULT_TRANSITION_TIME_IN_MS,   // Default transition time for models of the element in milliseconds
        .onpowerup_state = WICED_BT_MESH_ON_POWER_UP_STATE_RESTORE,      // Default element behavior on power up
        .default_level = 0,                                              // Default value of the variable controlled on this element (for example power, lightness, temperature, hue...)
        .range_min = 1,                                                  // Minimum value of the variable controlled on this element (for example power, lightness, temperature, hue...)
        .range_max = 0xffff,                                             // Maximum value of the variable controlled on this element (for example power, lightness, temperature, hue...)
        .move_rollover = 0,                                              // If true when level gets to range_max during move operation, it switches to min, otherwise move stops.
        .properties_num = 0,                                             // Number of properties in the array models
        .properties = NULL,                                              // Array of properties in the element.
        .sensors_num = 1,                                                // Number of properties in the array models
        .sensors = mesh_element2_sensors,                                // Array of properties in the element.
        .models_num = MESH_APP_NUM_MODELS_RED,                            // Number of models in the array models
        .models = mesh_element2_models,                                  // Array of models located in that element. Model data is defined by structure wiced_bt_mesh_core_config_model_t
    },
};

wiced_bt_mesh_core_config_t  mesh_config =
{
    .company_id         = MESH_COMPANY_ID_CYPRESS,                  // Company identifier assigned by the Bluetooth SIG
    .product_id         = MESH_PID,                                 // Vendor-assigned product identifier
    .vendor_id          = MESH_VID,                                 // Vendor-assigned product version identifier
    .firmware_id        = MESH_FWID,                                // Vendor-assigned firmware version identifier
    .replay_cache_size  = MESH_CACHE_REPLAY_SIZE,                   // Number of replay protection entries, i.e. maximum number of mesh devices that can send application messages to this device.
#if defined(LOW_POWER_NODE) && (LOW_POWER_NODE == 1)
    .features           = WICED_BT_MESH_CORE_FEATURE_BIT_LOW_POWER, // A bit field indicating the device features. In Low Power mode no Relay, no Proxy and no Friend
    .friend_cfg         =                                           // Empty Configuration of the Friend Feature
    {
        .receive_window = 0,                                        // Receive Window value in milliseconds supported by the Friend node.
        .cache_buf_len  = 0,                                        // Length of the buffer for the cache
        .max_lpn_num    = 0                                         // Max number of Low Power Nodes with established friendship. Must be > 0 if Friend feature is supported.
    },
    .low_power          =                                           // Configuration of the Low Power Feature
    {
        .rssi_factor           = 2,                                 // contribution of the RSSI measured by the Friend node used in Friend Offer Delay calculations.
        .receive_window_factor = 2,                                 // contribution of the supported Receive Window used in Friend Offer Delay calculations.
        .min_cache_size_log    = 3,                                 // minimum number of messages that the Friend node can store in its Friend Cache.
        .receive_delay         = 100,                               // Receive delay in 1 ms units to be requested by the Low Power node.
        .poll_timeout          = 36000                              // Poll timeout in 100ms units to be requested by the Low Power node.
    },
#else
    .features           = WICED_BT_MESH_CORE_FEATURE_BIT_FRIEND | WICED_BT_MESH_CORE_FEATURE_BIT_RELAY | WICED_BT_MESH_CORE_FEATURE_BIT_GATT_PROXY_SERVER,   // In Friend mode support friend, relay
    .friend_cfg         =                                           // Configuration of the Friend Feature(Receive Window in Ms, messages cache)
    {
        .receive_window        = 200,
        .cache_buf_len         = 300,                               // Length of the buffer for the cache
        .max_lpn_num           = 4                                  // Max number of Low Power Nodes with established friendship. Must be > 0 if Friend feature is supported.
    },
    .low_power          =                                           // Configuration of the Low Power Feature
    {
        .rssi_factor           = 0,                                 // contribution of the RSSI measured by the Friend node used in Friend Offer Delay calculations.
        .receive_window_factor = 0,                                 // contribution of the supported Receive Window used in Friend Offer Delay calculations.
        .min_cache_size_log    = 0,                                 // minimum number of messages that the Friend node can store in its Friend Cache.
        .receive_delay         = 0,                                 // Receive delay in 1 ms units to be requested by the Low Power node.
        .poll_timeout          = 0                                  // Poll timeout in 100ms units to be requested by the Low Power node.
    },
#endif
    .gatt_client_only          = WICED_FALSE,                       // Can connect to mesh over GATT or ADV
    .elements_num  = (uint8_t)(sizeof(mesh_elements) / sizeof(mesh_elements[0])),   // number of elements on this device
    .elements      = mesh_elements                                  // Array of elements for this device
};

/*
 * Mesh application library will call into application functions if provided by the application.
 */
wiced_bt_mesh_app_func_table_t wiced_bt_mesh_app_func_table =
{
    mesh_app_init,                  // application initialization
    NULL,                           // Default SDK platform button processing
    NULL,                           // GATT connection status
    NULL,                           // attention processing
    mesh_app_notify_period_set,     // notify period set
    NULL,                           // WICED HCI command
    mesh_app_lpn_sleep,             // LPN sleep
    mesh_app_factory_reset,         // factory reset
};

 /******************************************************
 *               Function Definitions
 ******************************************************/
void mesh_app_init(wiced_bool_t is_provisioned)
{
#if 0
    extern uint8_t wiced_bt_mesh_model_trace_enabled;
    wiced_bt_mesh_model_trace_enabled = WICED_TRUE;
#endif
    uint32_t        cur_time = wiced_bt_mesh_core_get_tick_count();
    wiced_result_t  result;
    wiced_bt_mesh_core_config_sensor_t *p_sensor;
    wiced_bt_mesh_core_config_sensor_t *p_red_sensor;


#if 1

    extern uint8_t wiced_bt_mesh_model_trace_enabled;
    wiced_bt_mesh_model_trace_enabled = WICED_TRUE;

#endif

    wiced_bt_cfg_settings.device_name = (uint8_t *)" Temp + Light Sensor";
    wiced_bt_cfg_settings.gatt_cfg.appearance = APPEARANCE_SENSOR_TEMPERATURE;

    // Adv Data is fixed. Spec allows to put URI, Name, Appearance and Tx Power in the Scan Response Data.
    if (!is_provisioned)
    {
        wiced_bt_ble_advert_elem_t  adv_elem[3];
        uint8_t                     buf[2];
        uint8_t                     num_elem = 0;

        adv_elem[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
        adv_elem[num_elem].len         = (uint16_t)strlen((const char*)wiced_bt_cfg_settings.device_name);
        adv_elem[num_elem].p_data      = wiced_bt_cfg_settings.device_name;
        num_elem++;

        adv_elem[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_APPEARANCE;
        adv_elem[num_elem].len         = 2;
        buf[0]                         = (uint8_t)wiced_bt_cfg_settings.gatt_cfg.appearance;
        buf[1]                         = (uint8_t)(wiced_bt_cfg_settings.gatt_cfg.appearance >> 8);
        adv_elem[num_elem].p_data      = buf;
        num_elem++;

        wiced_bt_mesh_set_raw_scan_response_data(num_elem, adv_elem);
    }

    p_sensor = &mesh_config.elements[MESH_SENSOR_SERVER_ELEMENT_INDEX].sensors[MESH_TEMPERATURE_SENSOR_INDEX];
    p_red_sensor = &mesh_config.elements[MESH_SENSOR_SERVER_ELEMENT_INDEX].sensors[MESH_RED_SENSOR_INDEX];

    mesh_prop_fw_version[0] = 0x30 + (WICED_SDK_MAJOR_VER / 10);
    mesh_prop_fw_version[1] = 0x30 + (WICED_SDK_MAJOR_VER % 10);
    mesh_prop_fw_version[2] = 0x30 + (WICED_SDK_MINOR_VER / 10);
    mesh_prop_fw_version[3] = 0x30 + (WICED_SDK_MINOR_VER % 10);
    mesh_prop_fw_version[4] = 0x30 + (WICED_SDK_REV_NUMBER / 10);
    mesh_prop_fw_version[5] = 0x30 + (WICED_SDK_REV_NUMBER % 10);
    mesh_prop_fw_version[6] = 0x30 + (WICED_SDK_BUILD_NUMBER / 10);
    mesh_prop_fw_version[7] = 0x30 + (WICED_SDK_BUILD_NUMBER % 10);

    if (!is_provisioned)
        return;

    // When we are coming out of HID OFF and if we are provisioned, need to send data

    thermistor_init();

     // read the initial temperature
    mesh_sensor_current_temperature = mesh_sensor_get_temperature_8();

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

    /* Init the Light Sensoer Shield -*/
    CN0397_Init();

    // initialize the cadence timer.  Need a timer for each element because each sensor model can be
    // configured for different publication period.  This app has only one sensor.
    wiced_init_timer(&mesh_sensor_cadence_timer,
    				 &mesh_sensor_publish_timer_callback,
					 (TIMER_PARAM_TYPE)&mesh_config.elements[MESH_SENSOR_SERVER_ELEMENT_INDEX].sensors[MESH_TEMPERATURE_SENSOR_INDEX],
					 WICED_MILLI_SECONDS_TIMER);

    //restore the cadence from NVRAM
    wiced_hal_read_nvram(MESH_TEMPERATURE_SENSOR_CADENCE_NVRAM_ID, sizeof(wiced_bt_mesh_sensor_config_cadence_t), (uint8_t*)(&p_sensor->cadence), &result);

    // initialize the cadence timer for red sensor.  Need a timer for each element because each sensor model can be
    // configured for different publication period.
    wiced_init_timer(&mesh_light_sensor_cadence_timer,
    				 &mesh_red_sensor_publish_timer_callback,
					 (TIMER_PARAM_TYPE)&mesh_config.elements[MESH_SENSOR_SERVER_ELEMENT_INDEX].sensors[MESH_RED_SENSOR_INDEX],
					 WICED_MILLI_SECONDS_TIMER);

    //restore the cadence from NVRAM
    wiced_hal_read_nvram(MESH_TEMPERATURE_SENSOR_CADENCE_NVRAM_ID, sizeof(wiced_bt_mesh_sensor_config_cadence_t), (uint8_t*)(&p_sensor->cadence), &result);

    //restore the cadence from NVRAM
    wiced_hal_read_nvram(MESH_RED_SENSOR_CADENCE_VSID, sizeof(wiced_bt_mesh_sensor_config_cadence_t), (uint8_t*)(&p_red_sensor->cadence), &result);

    // init the 2 sensor servers - temp and red light sensor
    wiced_bt_mesh_model_sensor_server_init(MESH_TEMPERATURE_SENSOR_INDEX , mesh_temp_sensor_server_report_handler, mesh_sensor_server_config_change_handler, is_provisioned);
    wiced_bt_mesh_model_sensor_server_init(MESH_RED_SENSOR_INDEX, mesh_red_sensor_server_report_handler, mesh_sensor_server_config_change_handler, is_provisioned);

    mesh_sensor_sent_value = mesh_sensor_current_temperature;
    mesh_sensor_sent_time  = cur_time;

    // init the sensor server data */
    WICED_BT_TRACE("Pub value:%d time:%d\n", mesh_sensor_sent_value, mesh_sensor_sent_time);
    wiced_bt_mesh_model_sensor_server_data(MESH_SENSOR_SERVER_ELEMENT_INDEX, WICED_BT_MESH_PROPERTY_PRESENT_AMBIENT_TEMPERATURE, NULL);

    // Measure the light sensor and update it to mesh config
    CN0397_ReadData(RED, &mesh_sensor_red_sensor, &mesh_sensor_green_sensor, &mesh_sensor_blue_sensor);
    mesh_light_sensor_current_value = mesh_sensor_red_sensor * 10000;
   /* format data to send */
    mesh_red_sensor_sent_value[0] = (uint8_t)( (mesh_light_sensor_current_value>> 16) & 0xFF);
    mesh_red_sensor_sent_value[1] = (uint8_t)(( mesh_light_sensor_current_value>> 8) & 0xFF);;
    mesh_red_sensor_sent_value[2] = (uint8_t)( mesh_light_sensor_current_value &0xFF);
    wiced_bt_mesh_model_sensor_server_data(MESH_RED_SENSOR_INDEX, WICED_BT_MESH_PROPERTY_PRESENT_AMBIENT_LIGHT_LEVEL , NULL);
}

/*
 * New publication period is set. If it is for the sensor model, this application should take care of it.
 * The period may need to be adjusted based on the divisor.
 */
wiced_bool_t mesh_app_notify_period_set(uint8_t element_idx, uint16_t company_id, uint16_t model_id, uint32_t period)
{
    if ((element_idx != MESH_TEMPERATURE_SENSOR_INDEX) || (company_id != MESH_COMPANY_ID_BT_SIG) || (model_id != WICED_BT_MESH_CORE_MODEL_ID_SENSOR_SRV))
    {
        return WICED_FALSE;
    }
    mesh_sensor_publish_period = period;
    WICED_BT_TRACE("Sensor data send period:%dms\n", mesh_sensor_publish_period);
    mesh_sensor_server_restart_timer(&mesh_config.elements[element_idx].sensors[MESH_TEMPERATURE_SENSOR_INDEX]);
    return WICED_TRUE;
}

/*
 * Application is notified that core enters LPN mode.
 */
void mesh_app_lpn_sleep(uint32_t timeout_ms)
{
#if defined(LOW_POWER_NODE) && (LOW_POWER_NODE == 1)
    if (wiced_sleep_enter_hid_off(timeout_ms, 0, 0) != WICED_SUCCESS)
    {
        WICED_BT_TRACE("Entering HID-Off failed\n\r");
    }
#endif
}

/*
 * Application is notified that factory reset is executed.
 */
void mesh_app_factory_reset(void)
{
    wiced_hal_delete_nvram(MESH_TEMPERATURE_SENSOR_CADENCE_NVRAM_ID, NULL);
}

/*
 * Start periodic timer depending on the publication period, fast cadence divisor and minimum interval
 */
void mesh_sensor_server_restart_timer(wiced_bt_mesh_core_config_sensor_t *p_sensor)
{
    // If there are no specific cadence settings, publish every publish period.
    uint32_t timeout = mesh_sensor_publish_period;

    wiced_stop_timer(&mesh_sensor_cadence_timer);
    if (mesh_sensor_publish_period == 0)
    {
        // The thermistor is not interrupt driven.  If client configured sensor to send notification when
        // the value changes, we will need to check periodically if the condition has been satisfied.
        // The cadence.min_interval can be used because we do not need to send data more often than that.
        if ((p_sensor->cadence.min_interval != 0) &&
            ((p_sensor->cadence.trigger_delta_up != 0) || (p_sensor->cadence.trigger_delta_down != 0)))
        {
            timeout = p_sensor->cadence.min_interval;
        }
        else
        {
            WICED_BT_TRACE("sensor restart timer period:%d\n", mesh_sensor_publish_period);
            return;
        }
    }
    else
    {
        // If fast cadence period divisor is set, we need to check temperature more
        // often than publication period.  Publish if measurement is in specified range
        if (p_sensor->cadence.fast_cadence_period_divisor > 1)
        {
            mesh_sensor_fast_publish_period = mesh_sensor_publish_period / p_sensor->cadence.fast_cadence_period_divisor;
            timeout = mesh_sensor_fast_publish_period;
        }
        else
        {
            mesh_sensor_fast_publish_period = 0;
        }
        // The thermistor is not interrupt driven.  If client configured sensor to send notification when
        // the value changes, we may need to check value more often not to miss the trigger.
        // The cadence.min_interval can be used because we do not need to send data more often than that.
        if ((p_sensor->cadence.min_interval < timeout) &&
            ((p_sensor->cadence.trigger_delta_up != 0) || (p_sensor->cadence.trigger_delta_down != 0)))
        {
            timeout = p_sensor->cadence.min_interval;
        }
    }
    WICED_BT_TRACE("sensor restart timer:%d\n", timeout);
    wiced_start_timer(&mesh_sensor_cadence_timer, timeout);
}

/*
 * Helper function to read temperature from the thermistor and convert temperature in celsius
 * to Temperature 8 format.  Unit is degree Celsius with a resolution of 0.5. Minimum: -64.0 Maximum: 63.5.
 */
int8_t mesh_sensor_get_temperature_8(void)
{
    int16_t temp_celsius_100 = thermistor_read();

    if (temp_celsius_100 < -6400)
    {
        return 0x80;
    }
    else if (temp_celsius_100 >= 6350)
    {
        return 0x7F;
    }
    else
    {
        return (int8_t)(temp_celsius_100 / 50);
    }
}

/*
 * Process the configuration changes set by the Sensor Client.
 */
//void mesh_sensor_server_config_change_handler(uint8_t element_idx, uint16_t event, void *p_data)
void mesh_sensor_server_config_change_handler(uint8_t element_idx, uint16_t event, uint16_t property_id, uint16_t setting_property_id)
{
#if defined HCI_CONTROL
    wiced_bt_mesh_hci_event_t *p_hci_event;
#endif
    WICED_BT_TRACE("mesh_sensor_server_config_change_handler msg: %d\n", event);

    switch (event)
    {

    case WICED_BT_MESH_SENSOR_CADENCE_SET:
#if defined HCI_CONTROL
//        if ((p_hci_event = wiced_bt_mesh_create_hci_event(p_event)) != NULL)
//            mesh_sensor_hci_event_send_cadence_set(p_hci_event, (wiced_bt_mesh_sensor_cadence_set_data_t *)p_data);
#endif
        mesh_sensor_server_process_cadence_changed(element_idx, property_id);
        break;

    case WICED_BT_MESH_SENSOR_SETTING_SET:
#if defined HCI_CONTROL
//        if ((p_hci_event = wiced_bt_mesh_create_hci_event(p_event)) != NULL)
//            mesh_sensor_hci_event_send_setting_set(p_hci_event, (wiced_bt_mesh_sensor_setting_set_data_t *)p_data);
#endif
        mesh_sensor_server_process_setting_changed(element_idx, property_id, setting_property_id);
        break;
    }
}

/*
 * Process get request from Sensor Client and respond with sensor data
 */
void mesh_temp_sensor_server_report_handler(uint16_t event, uint8_t element_idx, void *p_get, void *p_ref_data)
{
    wiced_bt_mesh_sensor_get_t *p_sensor_get = (wiced_bt_mesh_sensor_get_t *)p_get;
    WICED_BT_TRACE("mesh_sensor_server_report_handler msg: %d\n", event);

    switch (event)
    {
    case WICED_BT_MESH_SENSOR_GET:
        // measure the temperature and update it to mesh_config
        mesh_sensor_sent_value = mesh_sensor_get_temperature_8();

        // tell mesh models library that data is ready to be shipped out, the library will get data from mesh_config
        wiced_bt_mesh_model_sensor_server_data(element_idx, p_sensor_get->property_id, p_ref_data);

        break;

    case WICED_BT_MESH_SENSOR_COLUMN_GET:
        break;

    case WICED_BT_MESH_SENSOR_SERIES_GET:
        break;

    default:
        WICED_BT_TRACE("unknown\n");
        break;
    }
}

void mesh_red_sensor_server_report_handler(uint16_t event, uint8_t element_idx, void *p_get, void *p_ref_data)
{
    wiced_bt_mesh_sensor_get_t *p_sensor_get = (wiced_bt_mesh_sensor_get_t *)p_get;
    WICED_BT_TRACE("mesh_sensor_server_report_handler msg: %d\n", event);

    switch (event)
    {
    case WICED_BT_MESH_SENSOR_GET:

        // Measure the light sensor and update it to mesh config
        CN0397_ReadData(RED, &mesh_sensor_red_sensor, &mesh_sensor_green_sensor, &mesh_sensor_blue_sensor);
        mesh_light_sensor_current_value = mesh_sensor_red_sensor * 10000;

 //      WICED_BT_TRACE("Light sensor value: %x\n\r", mesh_light_sensor_current_value);

  //      CN0397_DisplayData();
       /* format data to send */
        mesh_red_sensor_sent_value[0] = (uint8_t)( (mesh_light_sensor_current_value>> 16) & 0xFF);
        mesh_red_sensor_sent_value[1] = (uint8_t)(( mesh_light_sensor_current_value>> 8) & 0xFF);;
        mesh_red_sensor_sent_value[2] = (uint8_t)( mesh_light_sensor_current_value &0xFF);

        // tell mesh models library that data is ready to be shipped out, the library will get data from mesh_config
        wiced_bt_mesh_model_sensor_server_data(element_idx, p_sensor_get->property_id, p_ref_data);

        break;

    case WICED_BT_MESH_SENSOR_COLUMN_GET:
        break;

    case WICED_BT_MESH_SENSOR_SERIES_GET:
        break;

    default:
        WICED_BT_TRACE("unknown\n");
        break;
    }
}

/*
 * Process cadence change
 */
void mesh_sensor_server_process_cadence_changed(uint8_t element_idx, uint16_t property_id)
{
    wiced_bt_mesh_core_config_sensor_t *p_sensor;
    uint8_t written_byte = 0;
    wiced_result_t status;
    p_sensor = &mesh_config.elements[element_idx].sensors[MESH_TEMPERATURE_SENSOR_INDEX];

    WICED_BT_TRACE("cadence changed property id:%04x\n", property_id);
    WICED_BT_TRACE("Fast cadence period divisor:%d\n", p_sensor->cadence.fast_cadence_period_divisor);
    WICED_BT_TRACE("Is trigger type percent:%d\n", p_sensor->cadence.trigger_type_percentage);
    WICED_BT_TRACE("Trigger delta up:%d\n", p_sensor->cadence.trigger_delta_up);
    WICED_BT_TRACE("Trigger delta down:%d\n", p_sensor->cadence.trigger_delta_down);
    WICED_BT_TRACE("Min Interval:%d\n", p_sensor->cadence.min_interval);
    WICED_BT_TRACE("Fast cadence low:%d\n", p_sensor->cadence.fast_cadence_low);
    WICED_BT_TRACE("Fast cadence high:%d\n", p_sensor->cadence.fast_cadence_high);

    /* save cadence to NVRAM */
    written_byte = wiced_hal_write_nvram(MESH_TEMPERATURE_SENSOR_CADENCE_NVRAM_ID, sizeof(wiced_bt_mesh_sensor_config_cadence_t), (uint8_t*)(&p_sensor->cadence), &status);
    WICED_BT_TRACE("NVRAM write: %d\n", written_byte);

    mesh_sensor_server_restart_timer(p_sensor);
}

/*
 * Publication timer callback.  Need to send data if publish period expired, or
 * if value has changed more than specified in the triggers, or if value is in range
 * of fast cadence values.
 */
void mesh_sensor_publish_timer_callback(TIMER_PARAM_TYPE arg)
{
    wiced_bt_mesh_event_t *p_event;
    wiced_bt_mesh_core_config_sensor_t *p_sensor = (wiced_bt_mesh_core_config_sensor_t *)arg;
    wiced_bool_t pub_needed = WICED_FALSE;
    uint32_t cur_time = wiced_bt_mesh_core_get_tick_count();

    mesh_sensor_current_temperature = mesh_sensor_get_temperature_8();

    if ((cur_time - mesh_sensor_sent_time) < p_sensor->cadence.min_interval)
    {
        WICED_BT_TRACE("time since last pub:%d interval:%d\n", cur_time - mesh_sensor_sent_time, p_sensor->cadence.min_interval);
        wiced_start_timer(&mesh_sensor_cadence_timer, p_sensor->cadence.min_interval - cur_time + mesh_sensor_sent_time);
    }
    else
    {
        // check if publication timer expired
        if ((mesh_sensor_publish_period != 0) && (cur_time - mesh_sensor_sent_time >= mesh_sensor_publish_period))
        {
            WICED_BT_TRACE("Pub needed period\n");
            pub_needed = WICED_TRUE;
        }
        // still need to send if publication timer has not expired, but triggers are configured, and value
        // changed too much
        if (!pub_needed && ((p_sensor->cadence.trigger_delta_up != 0) || (p_sensor->cadence.trigger_delta_down != 0)))
        {
            if (!p_sensor->cadence.trigger_type_percentage)
            {
                WICED_BT_TRACE("Native cur value:%d sent:%d delta:%d/%d\n",
                        mesh_sensor_current_temperature, mesh_sensor_sent_value, p_sensor->cadence.trigger_delta_up, p_sensor->cadence.trigger_delta_down);

                if (((p_sensor->cadence.trigger_delta_up != 0)   && (mesh_sensor_current_temperature >= (mesh_sensor_sent_value + p_sensor->cadence.trigger_delta_up))) ||
                    ((p_sensor->cadence.trigger_delta_down != 0) && (mesh_sensor_current_temperature <= (mesh_sensor_sent_value - p_sensor->cadence.trigger_delta_down))))
                {
                    WICED_BT_TRACE("Pub needed native value\n");
                    pub_needed = WICED_TRUE;
                }
            }
            else
            {
                // need to calculate percentage of the increase or decrease.  The deltas are in 0.01%.
                if ((p_sensor->cadence.trigger_delta_up != 0) && (mesh_sensor_current_temperature > mesh_sensor_sent_value))
                {
                    WICED_BT_TRACE("Delta up:%d\n", ((uint32_t)(mesh_sensor_current_temperature - mesh_sensor_sent_value) * 10000 / mesh_sensor_current_temperature));
                    if (((uint32_t)(mesh_sensor_current_temperature - mesh_sensor_sent_value) * 10000 / mesh_sensor_current_temperature) > p_sensor->cadence.trigger_delta_up)
                    {
                        WICED_BT_TRACE("Pub needed percent delta up:%d\n", ((mesh_sensor_current_temperature - mesh_sensor_sent_value) * 10000 / mesh_sensor_current_temperature));
                        pub_needed = WICED_TRUE;
                    }
                }
                else if ((p_sensor->cadence.trigger_delta_down != 0) && (mesh_sensor_current_temperature < mesh_sensor_sent_value))
                {
                    WICED_BT_TRACE("Delta down:%d\n", ((uint32_t)(mesh_sensor_sent_value - mesh_sensor_current_temperature) * 10000 / mesh_sensor_current_temperature));
                    if (((uint32_t)(mesh_sensor_sent_value - mesh_sensor_current_temperature) * 10000 / mesh_sensor_current_temperature) > p_sensor->cadence.trigger_delta_down)
                    {
                        WICED_BT_TRACE("Pub needed percent delta down:%d\n", ((mesh_sensor_sent_value - mesh_sensor_current_temperature) * 10000 / mesh_sensor_current_temperature));
                        pub_needed = WICED_TRUE;
                    }
                }
            }
        }
        // may still need to send if fast publication is configured
        if (!pub_needed && (mesh_sensor_fast_publish_period != 0))
        {
            // check if fast publish period expired
            if (cur_time - mesh_sensor_sent_time >= mesh_sensor_fast_publish_period)
            {
                // if cadence high is more than cadence low, to publish, the value should be in range
                if (p_sensor->cadence.fast_cadence_high >= p_sensor->cadence.fast_cadence_low)
                {
                    if ((mesh_sensor_current_temperature >= p_sensor->cadence.fast_cadence_low) &&
                        (mesh_sensor_current_temperature <= p_sensor->cadence.fast_cadence_high))
                    {
                        WICED_BT_TRACE("Pub needed in range\n");
                        pub_needed = WICED_TRUE;
                    }
                }
                else if (p_sensor->cadence.fast_cadence_high < p_sensor->cadence.fast_cadence_low)
                {
                    if ((mesh_sensor_current_temperature > p_sensor->cadence.fast_cadence_low) ||
                        (mesh_sensor_current_temperature < p_sensor->cadence.fast_cadence_high))
                    {
                        WICED_BT_TRACE("Pub needed out of range\n");
                        pub_needed = WICED_TRUE;
                    }
                }
            }
        }
        /*
        if (!pub_needed)
        {
           if (((p_sensor->cadence.trigger_delta_up == 0) && (mesh_sensor_current_temperature > mesh_sensor_sent_value)) ||
               ((p_sensor->cadence.trigger_delta_down == 0) && (mesh_sensor_current_temperature < mesh_sensor_sent_value)))
            {
               WICED_BT_TRACE("Pub needed new value no deltas\n");
               pub_needed = WICED_TRUE;
            }
        }
        */
        if (pub_needed)
        {
            mesh_sensor_sent_value  = mesh_sensor_current_temperature;
            mesh_sensor_sent_time   = cur_time;

            WICED_BT_TRACE("Pub value:%d time:%d\n", mesh_sensor_sent_value, mesh_sensor_sent_time);
            wiced_bt_mesh_model_sensor_server_data(MESH_SENSOR_SERVER_ELEMENT_INDEX, WICED_BT_MESH_PROPERTY_PRESENT_AMBIENT_TEMPERATURE, NULL);
        }
        mesh_sensor_server_restart_timer(p_sensor);
    }
}


void mesh_red_sensor_publish_timer_callback(TIMER_PARAM_TYPE arg)
{
    wiced_bt_mesh_event_t *p_event;
    wiced_bt_mesh_core_config_sensor_t *p_sensor = (wiced_bt_mesh_core_config_sensor_t *)arg;
    wiced_bool_t pub_needed = WICED_FALSE;
    uint32_t cur_time = wiced_bt_mesh_core_get_tick_count();

    CN0397_ReadData( RED, &mesh_sensor_red_sensor, &mesh_sensor_green_sensor, &mesh_sensor_blue_sensor);
    WICED_BT_TRACE("Mesh Red :%d  Mesh Green :%d Mesh Blue: %d\n", mesh_sensor_red_sensor, mesh_sensor_green_sensor, mesh_sensor_blue_sensor);

    mesh_red_sensor_current_value[0] = (uint8_t) mesh_sensor_red_sensor;
    mesh_red_sensor_current_value[1] = (uint8_t) mesh_sensor_green_sensor;
    mesh_red_sensor_current_value[2] = (uint8_t) mesh_sensor_blue_sensor;
     if ((cur_time - mesh_red_sensor_sent_time) < p_sensor->cadence.min_interval)
    {
        WICED_BT_TRACE("time since last pub:%d interval:%d\n", cur_time - mesh_red_sensor_sent_time, p_sensor->cadence.min_interval);
        wiced_start_timer(&mesh_sensor_cadence_timer, p_sensor->cadence.min_interval - cur_time + mesh_red_sensor_sent_time);
    }
    else
    {
        // check if publication timer expired
        if ((mesh_sensor_publish_period != 0) && (cur_time - mesh_red_sensor_sent_time >= mesh_sensor_publish_period))
        {
            WICED_BT_TRACE("Pub needed period\n");
            pub_needed = WICED_TRUE;
        }
        // still need to send if publication timer has not expired, but triggers are configured, and value
        // changed too much
        if (!pub_needed && ((p_sensor->cadence.trigger_delta_up != 0) || (p_sensor->cadence.trigger_delta_down != 0)))
        {
            if (!p_sensor->cadence.trigger_type_percentage)
            {
                WICED_BT_TRACE("Native cur value:%d sent:%d delta:%d/%d\n",
                		mesh_red_sensor_current_value[0], mesh_red_sensor_sent_value, p_sensor->cadence.trigger_delta_up, p_sensor->cadence.trigger_delta_down);

                if (((p_sensor->cadence.trigger_delta_up != 0)   && (mesh_red_sensor_current_value[0] >= (mesh_red_sensor_sent_value[0] + p_sensor->cadence.trigger_delta_up))) ||
                    ((p_sensor->cadence.trigger_delta_down != 0) && (mesh_red_sensor_current_value <= (mesh_red_sensor_sent_value - p_sensor->cadence.trigger_delta_down))))
               {
                    WICED_BT_TRACE("Pub needed native value\n");
                    pub_needed = WICED_TRUE;
                }
            }
//            else
//            {
//                // need to calculate percentage of the increase or decrease.  The deltas are in 0.01%.
//                if ((p_sensor->cadence.trigger_delta_up != 0) && (mesh_red_sensor_current_value > mesh_red_sensor_sent_value))
//                {
//                    WICED_BT_TRACE("Delta up:%d\n", ((uint32_t)(mesh_red_sensor_current_value - mesh_sensor_sent_value) * 10000 / mesh_red_sensor_current_value));
//                    if (((uint32_t)(mesh_red_sensor_current_value - mesh_red_sensor_sent_value) * 10000 / mesh_red_sensor_current_value) > p_sensor->cadence.trigger_delta_up)
//                    {
//                        WICED_BT_TRACE("Pub needed percent delta up:%d\n", ((mesh_red_sensor_current_value - mesh_red_sensor_sent_value) * 10000 / mesh_red_sensor_current_value));
//                        pub_needed = WICED_TRUE;
//                    }
//                }
//                else if ((p_sensor->cadence.trigger_delta_down != 0) && (mesh_red_sensor_current_value < mesh_red_sensor_sent_value))
//                {
//                    WICED_BT_TRACE("Delta down:%d\n", ((uint32_t)(mesh_sensor_sent_value - mesh_sensor_current_temperature) * 10000 / mesh_sensor_current_temperature));
//                    if (((uint32_t)(mesh_sensor_sent_value - mesh_red_sensor_current_value) * 10000 / mesh_red_sensor_current_value) > p_sensor->cadence.trigger_delta_down)
//                    {
//                        WICED_BT_TRACE("Pub needed percent delta down:%d\n", ((mesh_red_sensor_sent_value - mesh_red_sensor_current_value) * 10000 / mesh_red_sensor_current_value));
//                        pub_needed = WICED_TRUE;
//                    }
//                }
//            }
        }
        // may still need to send if fast publication is configured
        if (!pub_needed && (mesh_sensor_fast_publish_period != 0))
        {
            // check if fast publish period expired
            if (cur_time - mesh_red_sensor_sent_time >= mesh_sensor_fast_publish_period)
            {
                // if cadence high is more than cadence low, to publish, the value should be in range
                if (p_sensor->cadence.fast_cadence_high >= p_sensor->cadence.fast_cadence_low)
                {
                    if ((mesh_red_sensor_current_value[0] >= p_sensor->cadence.fast_cadence_low) &&
                        (mesh_red_sensor_current_value[0] <= p_sensor->cadence.fast_cadence_high))
                    {
                        WICED_BT_TRACE("Pub needed in range\n");
                        pub_needed = WICED_TRUE;
                    }
                }
                else if (p_sensor->cadence.fast_cadence_high < p_sensor->cadence.fast_cadence_low)
                {
                    if ((mesh_red_sensor_current_value[0] > p_sensor->cadence.fast_cadence_low) ||
                        (mesh_red_sensor_current_value[0] < p_sensor->cadence.fast_cadence_high))
                    {
                        WICED_BT_TRACE("Pub needed out of range\n");
                        pub_needed = WICED_TRUE;
                    }
                }
            }
        }

        /*
        if (!pub_needed)
        {
           if (((p_sensor->cadence.trigger_delta_up == 0) && (mesh_sensor_current_temperature > mesh_sensor_sent_value)) ||
               ((p_sensor->cadence.trigger_delta_down == 0) && (mesh_sensor_current_temperature < mesh_sensor_sent_value)))
            {
               WICED_BT_TRACE("Pub needed new value no deltas\n");
               pub_needed = WICED_TRUE;
            }
        }
        */
        if (pub_needed)
        {
        	mesh_red_sensor_sent_value[0]  = mesh_red_sensor_current_value[0];
        	mesh_red_sensor_sent_value[1]  = mesh_red_sensor_current_value[1];
        	mesh_red_sensor_sent_value[2]  = mesh_red_sensor_current_value[2];
            mesh_red_sensor_sent_time   = cur_time;

            WICED_BT_TRACE("Pub value:%d time:%d\n", mesh_sensor_sent_value, mesh_sensor_sent_time);
            wiced_bt_mesh_model_sensor_server_data(MESH_RED_SENSOR_INDEX, WICED_BT_MESH_PROPERTY_PRESENT_AMBIENT_LIGHT_LEVEL, NULL);
        }
        mesh_sensor_server_restart_timer(p_sensor);
    }
}
/*
 * Process setting change
 */
void mesh_sensor_server_process_setting_changed(uint8_t element_idx, uint16_t property_id, uint16_t setting_property_id)
{
    WICED_BT_TRACE("settings changed  property id of sensor = %x , sensor prop id = %x \n", property_id, setting_property_id);
}
