# RDK3 BLE Sensor Hub Code Example

Rutronik Development Kit 3 Programmable System-on-Chip CYB06447BZI-BLD53 "RDK3 Sensor Hub" Code Example. 

This code example demonstrates the Android OS “CySmart” application’s custom BLE service “Sensor Hub”. The accelerometer, barometer, thermometer, and battery voltage data may be monitored using this service.

The [RAB1-SENSORFUSION](https://www.rutronik24.com/product/rutronik/rab1-sensorfusion/19375973.html) demo board and Li-ION single-cell battery are needed additionally for this application to function properly.

<img src="images/rdk3_top.jpg" style="zoom:35%;" />

## Requirements

- [ModusToolbox™ software](https://www.cypress.com/products/modustoolbox-software-environment) v3.0

## The Provisioning of the RDK3

The CYB06447BZI-BLD53 device must be provisioned with keys and policies before being programmed. If the kit is already provisioned, copy-paste the keys and policy folder to the application folder. If the unsigned or not properly signed image will be written to the RDK3 PSoC™ 64 – the microcontroller will not boot. 

The “[Secure Policy Configurator](https://www.infineon.com/dgdl/Infineon-ModusToolbox_Secure_Policy_Configurator_1.30_User_Guide-UserManual-v01_00-EN.pdf?fileId=8ac78c8c8386267f0183a960762a5977)” tool is used for the provisioning of the new RDK3, please refer to the “ModusToolbox™ Secure Policy Configurator user guide”. 

The CYB06447BZI-BLD53 MCU must be powered from a 2.5V power source to be able to complete the provisioning. The RDK3 has an SMPS [Switching Mode Power Supply] which can be easily adjusted to provide 3.3V or 2.5V to the MCU by switching the slide-switch “SW1” on the bottom side of the board. 

<img src="images/voltage_switch.jpg" style="zoom:50%;" />

Please note that the “[Secure Policy Configurator](https://www.infineon.com/dgdl/Infineon-ModusToolbox_Secure_Policy_Configurator_1.30_User_Guide-UserManual-v01_00-EN.pdf?fileId=8ac78c8c8386267f0183a960762a5977) 1.20” requires the KitProg3 to be set into the CMSIS-DAP mode. Please press the “PROG MODE” button on the RDK3 board's front side once. The DEBUG D5 yellow led will flash indicating the CMSIS-MODE activated.

<img src="images/prog_mode.jpg" style="zoom:75%;" />

### Using the code example with a ModusToolbox™ IDE:

1. Import the project: **File** > **Import...** > **General** > **Existing Projects into Workspace** > **Next**.
2. Select the directory where **"RDK3_BLE_SensorHub"** resides and click  **Finish**.
3. Update the libraries using a **"Library Manager"** tool.
4. Select and build the project **Project ** > **Build Project**.

### Operation

The data is sent from the RDK3 server over BLE to the smartphone client app “CySmart” which is recognized as a "SensorHub" service with a particular accelerometer, barometer, thermometer, and battery level characteristics. The accelerometer is BMI270 from Bosch, the thermometer is SHT41 from Sensirion, and the barometer is DPS310 from Infineon. The "Read" and "Notify" methods are used to exchange sensor data with a client, hence the client may read all the values at any time and it is also notified about the changing values from the server side.

<img src="images/SensorHubDemo.jpg" style="zoom:50%;" />

### Debugging

If you successfully have imported the example, the debug configurations are already prepared to use with a the KitProg3, MiniProg4, or J-link. Open the ModusToolbox™ perspective and find the Quick Panel. Click on the desired debug launch configuration and wait for the programming to complete and the debugging process to start.

<img src="images/debug_start.png" style="zoom:100%;" />

## Legal Disclaimer

The evaluation board including the software is for testing purposes only and, because it has limited functions and limited resilience, is not suitable for permanent use under real conditions. If the evaluation board is nevertheless used under real conditions, this is done at one’s responsibility; any liability of Rutronik is insofar excluded. 

<img src="images/rutronik_origin_kaunas.png" style="zoom:50%;" />



