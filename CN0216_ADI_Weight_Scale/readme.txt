-------------------------------------------------------------------------------
CN0216 ADI Weight Scale Aruunio Shield
-------------------------------------------------------------------------------

Overview
--------
This code example requires a CYW92018EVB-02 and the CN0216 ADI
WEIght Scale Arduino Shield. - Precision Weigh Scale Design Using 
the AD7791 24-Bit Sigma-Delta ADC with External ADA4528-1 Zero-Drift 
Amplifiers 

Instructions to evaluate the project
---------------------------
To demonstrate the app, follow the steps:
1. Plug the CN0216 onto the Arduino connectors on the CYW920819EVB-02
2. Add a jumper witre on CN0216 - 5V to VIN on the Arduino Power Connector
3. Move the Jumper on Pxx to the 1st position
4. Plug the two WICED evaluation kits into your computer.
5. Build and download the application CN0216 Weight Scale" to thekit a
6. Use a terminal emulation tool such as Tera Term to open the serial port
   of WICED Peripheral UART with the below settings for both kits to view
   log/trace messages.
   [Baud rate: 115,200bps; Data: 8 bits; Parity: None; Stop bit: 1 bit]
7. See the weight being displayed in the TeraTerm window   
8. For more details on the hardware connections in the CYW920819EVB-02 Evaluation
   kit, please refer to the ModusToolbox CYW920819EVB-02 Evaluation Kit User Guide.pdf
   (http://www.cypress.com/CYW920819EVB-02)
9. For more information on the CN0216 Precisionn Weight Scale Design refer to documents 
   in this link   
   https://www.analog.com/en/design-center/reference-designs/circuits-from-the-lab/cn0216.html#rd-overview
   
-------------------------------------------------------------------------------