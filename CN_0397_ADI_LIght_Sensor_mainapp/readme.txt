-------------------------------------------------------------------------------
CN0397 ADI Light Sensor Arduino Shield Application
-------------------------------------------------------------------------------

Overview
--------
This application requires the CN0397 ADI Light Sensor Arduino Shield

To demonstrate the app, follow the steps:

1. Plug the CN0397 onto the Arduino Header
    SPI Master   
    Function   | WICED PLatform Name 	Arduino Header Name   
    CLK        | 	WICED_P09      			D13
    MISO       | 	WICED_P17      			D12
    MOSI       | 	WICED_P14      			D11
    CS         | 	WICED_P15      			D10
    GND        | 	GND                   

2. Use a terminal emulation tool such as Tera Term to open the serial port
   of WICED Peripheral UART with the below settings for both kits to view
   log/trace messages.
   [Baud rate: 115,200bps; Data: 8 bits; Parity: None; Stop bit: 1 bit]
   
3. On startup, the program calibrates the senssor.  You need to cover the sensors
   press SW3 to continue for the RED, GREEN and BLUE senosr.
  
4. The program then read the sensor and displasy Light Intensity and Concentraion
   for the RED, GREEN and BLUE sensors. 
  
   For more information on the CN0397 ADI Light Sensor Shield refer to the following
   https://www.analog.com/en/design-center/reference-designs/hardware-reference-design/circuits-from-the-lab/CN0397.html
      
   For more information on the AD7798 - 3 channel, low noise, 24 bit Delta Sigma ADC 
   used on the CN0397, please refer to
   https://www.analog.com/en/products/ad7798.html

   For more details on the hardware connections in the CYW920819EVB-02 Evaluation
   kit, please refer to the ModusToolbox CYW920819EVB-02 Evaluation Kit User Guide.pdf
   (http://www.cypress.com/CYW920819EVB-02)

-------------------------------------------------------------------------------

Edited - 8/30/2019