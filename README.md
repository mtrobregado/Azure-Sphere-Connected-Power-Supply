   
   Project Name: Azure Sphere Connected Power Supply
   Sphere OS: 19.05
   This file contains the 'main' function. Program execution begins and ends there
   AvnetStarterKitOLED

   Author: Markel T. Robregado
   Date: 10/1/2019

   Reference:
   The original project is based from AvnetStarterKitOLED.sln by 
   Peter Fenn (Avnet Engineering & Technology)
   Brian Willess (Avnet Engineering & Technology)

   Project Components:

   1. RDTech DPS3005 Power Supply.
   2. RDTech DPS3005 Communication Version Housing with fan.
   3. Mosfet Switch Board.
   4. Jumper Wires.

   Purpose:
   Using the Avnet Azure Sphere Starter Kit demonstrate a connected power supply with 
   following features.

   1. Read the temperature from LSM6DSO device using the I2C Interface
   2. Using UART Modbus RTU read DSP3005 PSU: 
		- Output Voltage Display Value 
		- Output Current Display Value
		- Output Power Display Value
		- Input Voltage Display Value

   3. Using UART Modbus RTU write to DPS3005 PSU:
		- Voltage Setting
		- Current Setting
		- Switch Output State [0 - 1]
		- Backlight Led [0 - 5]
   4. Using GPIO 17 trigger MOSFET switch board to turn on fan.   
   5. Read BSSID address, Wi-Fi AP SSID, Wi-Fi Frequency

   *************************************************************************************************
      Connected Power Supply: When connected to IoT Central
   *************************************************************************************************
   6. Send temperature from LSM6DSO device Azure IoT Central.
   7. Send DPS3005 PSU data to Azure IoT Central:
		- Output Voltage Display Value 
		- Output Current Display Value
		- Output Power Display Value
		- Input Voltage Display Value
   8. From Azure IOT Central set DPS3005 PSU using device twin properties:
		- Voltage Setting
		- Current Setting
		- Switch Output State [0 - 1]
		- Backlight Led [0 - 5]
   9. From Azure IOT Central set Connected Power Supply Temperature Threshold.
      If Connected Power Supply Temperature is greater than Threshold fan will
	  turn on using GPIO 17.
   9. Send button state data to Azure
   10. Send BSSID address, Wi-Fi AP SSID, Wi-Fi Frequency data to Azure IOT Central.
   11. Send the application version string to Azure IOT Central.