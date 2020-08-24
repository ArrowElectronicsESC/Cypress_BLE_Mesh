-------------------------------------------------------------------------------
BLE Mesh Sensor Temperature application
-------------------------------------------------------------------------------

Overview
--------
This demo application shows an implementation of a BLE Mesh temperature sensor.
The app is based on the BLE Mesh Sensor Server model.

Features demonstrated
- Temperature measurement using the on board Thermistor on the CYBT-213043-MESH EZ-BT Mesh Evaluation Kit
  / CYW920819EVB-02 Evaluation Kit
- Usage of BLE Mesh Sensor Server model

See chip specific readme for more information about the BT SDK.

Instructions
------------
To demonstrate the app, work through the following steps.
1. Build and download the application to the CYBT-213043-MESH EZ-BT Mesh Evaluation Kit
   / CYW920819EVB-02 Evaluation Kit
2. Use Android MeshController or Windows Mesh Client and provision the temperature sensor
3. After successful provisioning, use the Android MeshController/ Windows Mesh
   Client to configure the below parameters of the sensor
     a> configure sensor to publish the sensor data to a group(all-nodes, all-relays).
     b> configure publish period : publish period defines how often the user wants the
        sensor to publish the data. For testing, you can set it to 5000 msec.
     c> set cadence of the sensor (optional step):
        set minimum interval in which sensor data has to be published.
        set the range in which the fast cadence has to be observed.
        set the fast cadence period (how fast the data has to be published with respect
        to publish period).
        set the unit in which if the values change the data should be published and
        trigger type (Native or percentage).
           example : publish data if the data changes by 2 units/10%
4. Observe the temperature value on the Android app / trace window in Mesh Client app.
5. To change the temperature on the thermistor, you can keep your finger on the onboard
   sensor and see the changes.

Notes
-----
The application GATT database is located in -
bt_sdk-1.x\components\BT-SDK\common\libraries\mesh_app_lib\mesh_app_gatt.c
If you create a GATT database using Bluetooth Configurator, update the
GATT database in the location mentioned above.

Project Settings
----------------
Application specific project settings are as below -

MESH_MODELS_DEBUG_TRACES
   Turn on debug trace from Mesh Models library
MESH_CORE_DEBUG_TRACES
   Turn on debug trace from Mesh Core library
MESH_PROVISIONER_DEBUG_TRACES
   Turn on debug trace from Mesh Provisioner library
REMOTE_PROVISION_SRV
   Enable device as Remote Provisioning Server
LOW_POWER_NODE
   Enable device as Low Power Node

-----------------------------------------------------------------------

Template Code
------------------------------------------------

#include "wiced_hal_pspi.h"

Add Element 2

wiced_bt_mesh_core_config_model_t   mesh_element2_models[] =
{
		WICED_BT_MESH_MODEL_SENSOR_SERVER,
};
#define MESH_APP_NUM_MODELS_RED  (sizeof(mesh_element2_models) / sizeof(wiced_bt_mesh_core_config_model_t))


Red Sensor send variable

int8_t       mesh_red_sensor_sent_value[3] = {0,0,0};  // Ambient Light Sensor Element required 3 octets

Red Sensor Defines 

// The Red Sensor defines
#define MESH_RED_SENSOR_POSITIVE_TOLERANCE        	   	WICED_BT_MESH_SENSOR_TOLERANCE_UNSPECIFIED
#define MESH_RED_SENSOR_NEGATIVE_TOLERANCE          	WICED_BT_MESH_SENSOR_TOLERANCE_UNSPECIFIED

#define MESH_RED_SENSOR_SAMPLING_FUNCTION            	WICED_BT_MESH_SENSOR_SAMPLING_FUNCTION_UNKNOWN
#define MESH_RED_SENSOR_MEASUREMENT_PERIOD           	WICED_BT_MESH_SENSOR_VAL_UNKNOWN
#define MESH_RED_SENSOR_UPDATE_INTERVAL              	WICED_BT_MESH_SENSOR_VAL_UNKNOWN

#define MESH_RED_SENSOR_CADENCE_VSID	          		WICED_NVRAM_VSID_START + 0x100

/**  Add mesh_element_model_2_mode -  remember you only need WICED_BT_MESH_DEVICE on element1

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


Add the 2nd Sensor to the bt_mesh_core_config_element_t mesh_elements[] structure

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
    

	/* Init the SPI Hardware */

    wiced_hal_pspi_init(SPI1,
                        SPI_MASTER,
                        INPUT_PIN_PULL_UP,
                        GPIO_CFG(CS_1,CLK_1,MOSI_1,MISO_1),
                        DEFAULT_FREQUENCY,
                        SPI_MSB_FIRST,
                        SPI_SS_ACTIVE_LOW,
                        SPI_MODE_3,
                        CS_1);
                        
    /* Init the Light Sensor Shield -*/
    CN0397_Init();
    
    Add a WICED Timer for the Red Sensor publish
    
    // initialize the cadence timer for red sensor.  Need a timer for each element because each sensor model can be
    // configured for different publication period.
    wiced_init_timer(&mesh_light_sensor_cadence_timer,
    				 &mesh_red_sensor_publish_timer_callback,
					 (TIMER_PARAM_TYPE)&mesh_config.elements[MESH_SENSOR_SERVER_ELEMENT_INDEX].sensors[MESH_RED_SENSOR_INDEX],
					 WICED_MILLI_SECONDS_TIMER);
    
    //restore the red sensor cadence from NVRAM
    wiced_hal_read_nvram(MESH_RED_SENSOR_CADENCE_VSID, sizeof(wiced_bt_mesh_sensor_config_cadence_t), (uint8_t*)(&p_red_sensor->cadence), &result);
    
 
 	/* Add the MESH_RED_SENSOR_CADENCE_NVRAM_ID  #define here *
    #define MESH_RED_SENSOR_CADENCE_VSID	          		WICED_NVRAM_VSID_START + 0x100
    
    Init the red sensor server
    
       wiced_bt_mesh_model_sensor_server_init(MESH_RED_SENSOR_INDEX, mesh_red_sensor_server_report_handler, mesh_sensor_server_config_change_handler, is_provisioned);
    
                            
