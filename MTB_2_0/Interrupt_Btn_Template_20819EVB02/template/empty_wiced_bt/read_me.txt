-------------------------------------------------------------------------------
WICED Bluetooth Template Application
-------------------------------------------------------------------------------


Overview
--------
This application is provided as a template application for
CYW20xxx family of devices, a starting point for adding new code and
functionality. As provided, the app only outputs a trace message on
initialization of the bluetooth stack.

See chip specific readme for more information about the BT SDK.

Instructions
------------
To demonstrate the app, follow these steps -

1. Build and download the application to the eval board
2. Use Terminal emulation tools like Teraterm open to the WICED
   Peripheral UART serial port and observe trace message output. Usually
   baud rate is 115200 (application configurable).

Note
----

- See the TODO comments in the application code and fill in your application code.
- Use the 'Configure Device' option to configure your device periperials if needed.
- Use the 'Bluetooth Configurator' to configure application GATT database if needed.

-------------------------------------------------------------------------------

Example Code 

TODO #1

        	/* Configure the button to trigger an interrupt when pressed */
        	wiced_hal_gpio_configure_pin(WICED_GPIO_PIN_BUTTON_1, ( GPIO_INPUT_ENABLE | GPIO_PULL_UP | GPIO_EN_INT_FALLING_EDGE ), GPIO_PIN_OUTPUT_HIGH );
        	wiced_hal_gpio_register_pin_for_interrupt( WICED_GPIO_PIN_BUTTON_1, button_cback, 0 );
        	
TODO #2 

	/* Toggle the LED when this interrupt routine is entered */
	if( GPIO_PIN_OUTPUT_HIGH == wiced_hal_gpio_get_pin_output( WICED_GPIO_PIN_LED_1 ) )
	{
		wiced_hal_gpio_set_pin_output( WICED_GPIO_PIN_LED_1, GPIO_PIN_OUTPUT_LOW );
	}
	else
	{
		wiced_hal_gpio_set_pin_output( WICED_GPIO_PIN_LED_1, GPIO_PIN_OUTPUT_HIGH );
	}        	
      