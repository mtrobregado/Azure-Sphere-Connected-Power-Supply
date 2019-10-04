/* Copyright (c) Microsoft Corporation. All rights reserved.
   Licensed under the MIT License. */

   /************************************************************************************************
   Name: AvnetStarterKitReferenceDesign
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
   	 
   *************************************************************************************************/


/*********************************************************************
 * INCLUDES
 */
#include <errno.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h> 
#include <time.h>
#include <unistd.h>
#include <stdio.h>
#include <math.h>

// applibs_versions.h defines the API struct versions to use for applibs APIs.
#include "applibs_versions.h"
#include "epoll_timerfd_utilities.h"
#include "i2c.h"
#include "mt3620_avnet_dev.h"
#include "mt3620_rdb.h"
#include "deviceTwin.h"
#include "azure_iot_utilities.h"
#include "connection_strings.h"
#include "build_options.h"

#include <applibs/log.h>
#include <applibs/i2c.h>
#include <applibs/uart.h>
#include <applibs/gpio.h>
#include <applibs/wificonfig.h>
#include <azureiot/iothub_device_client_ll.h>

#include "main.h"
#include "dps3005.h"

/*********************************************************************
 * CONSTANTS
 */


/*********************************************************************
 * EXTERN VARIABLES
 */
// Provide local access to variables in other files
extern twin_t twinArray[];
extern int twinArraySize;
extern IOTHUB_DEVICE_CLIENT_LL_HANDLE iothubClientHandle;
extern float TempThreshold;


/*********************************************************************
 * LOCAL VARIABLES
 */
uint8_t  _u8MBSlave;                                         // Modbus slave (1..255) initialized in begin()
uint16_t _u16ReadAddress;                                    // slave register from which to read
uint16_t _u16ReadQty;                                        // quantity of words to read
uint16_t _u16ResponseBuffer[MAXBUFFERSIZE] = { 0, };                  // buffer to store Modbus slave response; read via GetResponseBuffer()
uint16_t _u16WriteAddress;                                   // slave register to which to write
uint16_t _u16WriteQty;                                       // quantity of words to write
uint16_t _u16TransmitBuffer[MAXBUFFERSIZE] = { 0, };                  // buffer containing data to transmit to Modbus slave; set via SetTransmitBuffer()
uint8_t u8ResponseBufferLength = 0;

// File descriptors - initialized to invalid value
int epollFd = -1;
static int uartFd = -1;
static int psupplyTimerFd = -1;
int fanenableGpioFd = -1;

int userLedRedFd = -1;
int userLedGreenFd = -1;
int userLedBlueFd = -1;
int appLedFd = -1;
int wifiLedFd = -1;
int clickSocket1Relay1Fd = -1;
int clickSocket1Relay2Fd = -1;

float VOut = 0.0; // Output Voltage Display Value
float AOut = 0.0; // Output Current Display Value
float POut = 0.0; // Output Power Display Value
float VIn = 0.0; // Output Input Display Value

volatile bool ModbusTransactionFlag = false;


#if (defined(IOT_CENTRAL_APPLICATION) || defined(IOT_HUB_APPLICATION))
	bool versionStringSent = false;
#endif

// Define the Json string format for the accelerator button press data
static const char cstrButtonTelemetryJson[] = "{\"%s\":\"%d\"}";

// Termination state
volatile sig_atomic_t terminationRequired = false;



/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint16_t ModbusMaster_crc16update(uint16_t crc, uint8_t a);
static uint16_t ModbusMaster_getResponseBuffer(uint8_t u8Index);
static uint8_t ModbusMaster_readHoldingRegisters(uint16_t u16ReadAddress, uint16_t u16ReadQty);
static uint8_t ModbusMaster_ModbusMasterSend(uint8_t u8MBFunction);
static uint8_t ModbusMaster_ModbusMasterReceive(uint8_t* data, uint8_t length);

static void TerminationHandler(int signalNumber);
static int InitPeripheralsAndHandlers(void);
static void ClosePeripheralsAndHandlers(void);

/******************************************************************************
 Modbus RTU Codes referenced from ModbusMaster.cpp, ModbusMaster.h
 - Arduino library for communicating with Modbus slaves by Doc Walker
 *****************************************************************************/
/*********************************************************************
 * @fn      ModbusMaster_crc16update
 *
 * @brief   .
 *
 * @param
 *
 * @return crc
 */
static uint16_t ModbusMaster_crc16update(uint16_t crc, uint8_t a)
{
	int i;

	crc ^= a;

	for (i = 0; i < 8; ++i)
	{
		if (crc & 1)
		{
			crc = (crc >> 1) ^ 0xA001;
		}
		else
		{
			crc = (crc >> 1);
		}
	}

	return crc;
}

/*********************************************************************
 * @fn      ModbusMaster_getResponseBuffer
 *
 * @brief   Retrieve data from response buffer.
 *
 * @param   u8Index
 *
 * @return  value in position u8Index of response buffer
 */
static void ModbusMaster_clearResponseBuffer(void)
{
	uint8_t i;

	for (i = 0; i < MAXBUFFERSIZE; i++)
	{
		_u16ResponseBuffer[i] = 0;
	}
}

/*********************************************************************
 * @fn      ModbusMaster_getResponseBuffer
 *
 * @brief   Retrieve data from response buffer.
 *
 * @param   u8Index
 *
 * @return  value in position u8Index of response buffer
 */
static uint16_t ModbusMaster_getResponseBuffer(uint8_t u8Index)
{
	if (u8Index < MAXBUFFERSIZE)
	{
		return _u16ResponseBuffer[u8Index];
	}
	else
	{
		return 0xFFFF;
	}
}

/*********************************************************************
 * @fn      ModbusMaster_readHoldingRegisters
 *
 * @brief   read the contents of a contiguous block of
 *          holding registers in a remote device
 *
 * @param   u16ReadAddress, u16ReadQty
 *
 * @return
 */
static uint8_t ModbusMaster_readHoldingRegisters(uint16_t u16ReadAddress, uint16_t u16ReadQty)
{
	_u16ReadAddress = u16ReadAddress;
	_u16ReadQty = u16ReadQty;

	return ModbusMaster_ModbusMasterSend(MB_READHOLDINGREGISTERS);
}

/*********************************************************************
 * @fn      ModbusMaster_writeSingleRegister
 *
 * @brief   write a single holding register in a
 *          remote device.
 *
 * @param   u16ReadAddress, u16WriteValue
 *
 * @return
 */
uint8_t ModbusMaster_writeSingleRegister(uint16_t u16WriteAddress, uint16_t u16WriteValue)
{
	_u16WriteAddress = u16WriteAddress;
	_u16TransmitBuffer[0] = u16WriteValue;

	return ModbusMaster_ModbusMasterSend(MB_WRITESINGLEREGISTER);
}

/*********************************************************************
 * @fn      ModbusMaster_ModbusMasterSend
 *
 * @brief   Assemble Modbus Request Application Data Unit(ADU). Transmit
 *          ADU using UART.
 *
 * @param   u8MBFunction
 *
 * @return
 */
static uint8_t ModbusMaster_ModbusMasterSend(uint8_t u8MBFunction)
{
	uint8_t u8ModbusADU[32] = { 0, };
	uint8_t u8ModbusADUSize = 0;
	uint8_t i;
	uint16_t u16CRC;
	uint8_t u8MBStatus = MB_SUCCESS;

	// assemble Modbus Request Application Data Unit
	u8ModbusADU[u8ModbusADUSize++] = MB_SLAVEADDRESS; // 0x01
	u8ModbusADU[u8ModbusADUSize++] = u8MBFunction; //0x03 //0x06

	switch (u8MBFunction)
	{

	case MB_READHOLDINGREGISTERS:
		u8ModbusADU[u8ModbusADUSize++] = _u16ReadAddress >> 8; // high byte
		u8ModbusADU[u8ModbusADUSize++] = _u16ReadAddress & 0x00FF; // low byte;
		u8ModbusADU[u8ModbusADUSize++] = _u16ReadQty >> 8; // high byte;
		u8ModbusADU[u8ModbusADUSize++] = _u16ReadQty & 0x00FF; // low byte;
		break;

	case MB_WRITESINGLEREGISTER:
		u8ModbusADU[u8ModbusADUSize++] = _u16WriteAddress >> 8; // high byte
		u8ModbusADU[u8ModbusADUSize++] = _u16WriteAddress & 0x00FF; // low byte
		u8ModbusADU[u8ModbusADUSize++] = _u16TransmitBuffer[0] >> 8; // high byte
		u8ModbusADU[u8ModbusADUSize++] = _u16TransmitBuffer[0] & 0x00FF; // low byte
		break;

	default:
		break;
	}

	// append CRC
	u16CRC = 0xFFFF;

	for (i = 0; i < u8ModbusADUSize; i++)
	{
		u16CRC = ModbusMaster_crc16update(u16CRC, u8ModbusADU[i]);
	}

	u8ModbusADU[u8ModbusADUSize++] = u16CRC & 0x00FF; // low byte crc
	u8ModbusADU[u8ModbusADUSize++] = u16CRC >> 8; // high byte crc
	u8ModbusADU[u8ModbusADUSize] = 0;

	write(uartFd, &u8ModbusADU[0], u8ModbusADUSize);

	u8ModbusADUSize = 0;

	return u8MBStatus;
}

/*********************************************************************
 * @fn      ModbusMaster_ModbusMasterReceive
 *
 * @brief   Assemble Modbus Request Application Data Unit(ADU). Transmit
 *          ADU using UART.
 *
 * @param   *data, length  
 *
 * @return
 */
static uint8_t ModbusMaster_ModbusMasterReceive(uint8_t* data, uint8_t length)
{
	uint8_t u8ModbusADU[32] = { 0, };
	uint8_t u8MBStatus = MB_SUCCESS;
	uint8_t i = 0;
	

	memcpy(&u8ModbusADU[0], data, length);

	// verify response is for correct Modbus slave
	if (u8ModbusADU[0] != MB_SLAVEADDRESS) //0x01
	{
		return MB_INVALIDSLAVEID;
	}

	if (u8ModbusADU[1] == MB_WRITESINGLEREGISTER)
	{
		ModbusTransactionFlag = false;
		return u8MBStatus;
	}

	if (u8ModbusADU[1] == MB_READHOLDINGREGISTERS)
	{
		
		// load bytes into word; response bytes are ordered H, L, H, L, ...
		for (i = 0; i < (u8ModbusADU[2] >> 1); i++)
		{
			if (i < MAXBUFFERSIZE)
			{
				_u16ResponseBuffer[i] = (((uint16_t)u8ModbusADU[2 * i + 3] << 8) | (uint16_t)u8ModbusADU[2 * i + 4]);
			}

			u8ResponseBufferLength = i;
		}

		VOut = ((float)ModbusMaster_getResponseBuffer(0) / 100); // get VOut from response buffer and convert to float
		AOut = ((float)ModbusMaster_getResponseBuffer(1) / 1000); // get AOut from response buffer and convert to float  
		POut = ((float)ModbusMaster_getResponseBuffer(2) / 100); // get POut from response buffer and convert to float 
		VIn = ((float)ModbusMaster_getResponseBuffer(3) / 100); // get VIn from response buffer and convert to float 
	
		Log_Debug("VOut: %.2f, ", VOut);
		Log_Debug("AOut: %.3f, ", AOut);
		Log_Debug("POut: %.2f, ", POut);
		Log_Debug("VIn: %.2f\r\n", VIn);	

		ModbusMaster_clearResponseBuffer();

		ModbusTransactionFlag = false;
		
#if (defined(IOT_CENTRAL_APPLICATION) || defined(IOT_HUB_APPLICATION))

		// We've seen that the first read of the Accelerometer data is garbage.  If this is the first pass
		// reading data, don't report it to Azure.  Since we're graphing data in Azure, this data point
		// will skew the data.
		

		// Allocate memory for a telemetry message to Azure
		char* pjsonBuffer = (char*)malloc(JSON_BUFFER_SIZE);
		if (pjsonBuffer == NULL) {
			Log_Debug("ERROR: not enough memory to send telemetry");
		}

		snprintf(pjsonBuffer, JSON_BUFFER_SIZE, "{\"vout\": \"%.2f\",\"aout\": \"%.2f\",\"pout\": \"%.2f\",\"vin\": \"%.2f\", \"temp\": \"%.2f\",  \"rssi\": \"%d\"}",
				VOut, AOut, POut, VIn, lsm6dsoTemperature_degC, network_data.rssi);


		//snprintf(pjsonBuffer, JSON_BUFFER_SIZE, "{ \"temp\": \"%.2f\",  \"rssi\": \"%d\"}", lsm6dsoTemperature_degC, network_data.rssi);
		Log_Debug("\n[Info] Sending telemetry: %s\n", pjsonBuffer);
		AzureIoT_SendMessage(pjsonBuffer);
		free(pjsonBuffer);
		
#endif
	}
	return u8MBStatus;
}

/// <summary>
///     Signal handler for termination requests. This handler must be async-signal-safe.
/// </summary>
static void TerminationHandler(int signalNumber)
{
    // Don't use Log_Debug here, as it is not guaranteed to be async-signal-safe.
    terminationRequired = true;
}

/// <summary>
///     Handle UART event: if there is incoming data, print it.
/// </summary>
static void UartEventHandler(EventData* eventData)
{
	const size_t receiveBufferSize = 32;
	uint8_t receiveBuffer[receiveBufferSize];
	ssize_t bytesRead;

	// Read incoming UART data. It is expected behavior that messages may be received in multiple
	// partial chunks.
	bytesRead = read(uartFd, receiveBuffer, receiveBufferSize);
	if (bytesRead < 0) {
		Log_Debug("ERROR: Could not read UART: %s (%d).\n", strerror(errno), errno);
		terminationRequired = true;
		return;
	}

	if (bytesRead > 0) {
		ModbusMaster_ModbusMasterReceive(receiveBuffer, (uint8_t)bytesRead);
	}
}

/// <summary>
///     Handle button timer event: if the button is pressed, report the event to the IoT Hub.
/// </summary>
static void PsupplyTimerEventHandler(EventData* eventData)
{
	if (ModbusTransactionFlag == false)
	{
		ModbusTransactionFlag = true;
		ModbusMaster_readHoldingRegisters(2, 5);  // slave: read a range of 16-bit registers starting at register 2 to 5  		
	}
}


// event handler data structures. Only the event handler field needs to be populated.
static EventData uartEventData = { .eventHandler = &UartEventHandler };
static EventData psupplyEventData = { .eventHandler = &PsupplyTimerEventHandler };

/// <summary>
///     Set up SIGTERM termination handler, initialize peripherals, and set up event handlers.
/// </summary>
/// <returns>0 on success, or -1 on failure</returns>
static int InitPeripheralsAndHandlers(void)
{
    struct sigaction action;
    memset(&action, 0, sizeof(struct sigaction));
    action.sa_handler = TerminationHandler;
    sigaction(SIGTERM, &action, NULL);

    epollFd = CreateEpollFd();
    if (epollFd < 0) {
        return -1;
    }

	// Create a UART_Config object, open the UART and set up UART event handler
	UART_Config uartConfig;
	UART_InitConfig(&uartConfig);
	uartConfig.baudRate = 9600;
	uartConfig.flowControl = UART_FlowControl_None;
	uartFd = UART_Open(AVT_SK_PMOD_ISU0_UART, &uartConfig);
	if (uartFd < 0) {
		Log_Debug("ERROR: Could not open UART: %s (%d).\n", strerror(errno), errno);
		return -1;
	}
	if (RegisterEventHandlerToEpoll(epollFd, uartFd, &uartEventData, EPOLLIN) != 0) {
		return -1;
	}

	ModbusMaster_clearResponseBuffer();	

	if (initI2c() == -1) {
		return -1;
	}

	// Open Fan Enable GPIO to triger the MOSFET Board to turn on fan
	Log_Debug("Opening FAN Enable GPIO as output.\n");
	fanenableGpioFd = GPIO_OpenAsOutput(MT3620_RDB_PMOD_GPIO17, GPIO_OutputMode_PushPull, GPIO_Value_Low);
	if (fanenableGpioFd < 0) {
		Log_Debug("ERROR: Could not open LED GPIO: %s (%d).\n", strerror(errno), errno);
		return -1;
	}
	
	// Traverse the twin Array and for each GPIO item in the list open the file descriptor
	for (int i = 0; i < twinArraySize; i++) {

		// Verify that this entry is a GPIO entry
		if (twinArray[i].twinGPIO != NO_GPIO_ASSOCIATED_WITH_TWIN) {

			*twinArray[i].twinFd = -1;

			// For each item in the data structure, initialize the file descriptor and open the GPIO for output.  Initilize each GPIO to its specific inactive state.
			*twinArray[i].twinFd = (int)GPIO_OpenAsOutput(twinArray[i].twinGPIO, GPIO_OutputMode_PushPull, twinArray[i].active_high ? GPIO_Value_Low : GPIO_Value_High);

			if (*twinArray[i].twinFd < 0) {
				Log_Debug("ERROR: Could not open LED %d: %s (%d).\n", twinArray[i].twinGPIO, strerror(errno), errno);
				return -1;
			}
		}
	}

	
	// Setup up a timer to periodically read the power supply
	struct timespec psupplyReadPeriod = { .tv_sec = PSUPPLY_READ_PERIOD_SECONDS,.tv_nsec = PSUPPLY_READ_PERIOD_NANO_SECONDS };
	psupplyTimerFd = CreateTimerFdAndAddToEpoll(epollFd, &psupplyReadPeriod, &psupplyEventData, EPOLLIN);
	if (psupplyTimerFd < 0) {
		return -1;
	}
		
	// Tell the system about the callback function that gets called when we receive a device twin update message from Azure
	AzureIoT_SetDeviceTwinUpdateCallback(&deviceTwinChangedHandler);

    return 0;
}

/// <summary>
///     Close peripherals and handlers.
/// </summary>
static void ClosePeripheralsAndHandlers(void)
{
    Log_Debug("Closing file descriptors.\n");
    
	closeI2c();
    CloseFdAndPrintError(epollFd, "Epoll");	
	CloseFdAndPrintError(uartFd, "isu0");
	CloseFdAndPrintError(psupplyTimerFd, "psupplytimer");	
	CloseFdAndPrintError(fanenableGpioFd, "gpio17");	

	// Traverse the twin Array and for each GPIO item in the list the close the file descriptor
	for (int i = 0; i < twinArraySize; i++) {

		// Verify that this entry has an open file descriptor
		if (twinArray[i].twinGPIO != NO_GPIO_ASSOCIATED_WITH_TWIN) {

			CloseFdAndPrintError(*twinArray[i].twinFd, twinArray[i].twinKey);
		}
	}
}

/// <summary>
///     Main entry point for this application.
/// </summary>
int main(int argc, char *argv[])
{
	// Variable to help us send the version string up only once
	bool networkConfigSent = false;
	char ssid[128];
	uint32_t frequency;
	char bssid[20];
	
	// Clear the ssid array
	memset(ssid, 0, 128);

	Log_Debug("Version String: %s\n", argv[1]);
	Log_Debug("Avnet Starter Kit Simple Reference Application starting.\n");
	
    if (InitPeripheralsAndHandlers() != 0) {
        terminationRequired = true;
    }

    // Use epoll to wait for events and trigger handlers, until an error or SIGTERM happens
    while (!terminationRequired) {
        if (WaitForEventAndCallHandler(epollFd) != 0) {
            terminationRequired = true;
        }

#if (defined(IOT_CENTRAL_APPLICATION) || defined(IOT_HUB_APPLICATION))
		// Setup the IoT Hub client.
		// Notes:
		// - it is safe to call this function even if the client has already been set up, as in
		//   this case it would have no effect;
		// - a failure to setup the client is a fatal error.
		if (!AzureIoT_SetupClient()) {
			Log_Debug("ERROR: Failed to set up IoT Hub client\n");
			break;
		}
#endif 

		WifiConfig_ConnectedNetwork network;
		int result = WifiConfig_GetCurrentNetwork(&network);

		if (result < 0) 
		{
			// Log_Debug("INFO: Not currently connected to a WiFi network.\n");
			//// OLED
			strncpy(network_data.SSID, "Not Connected", 20);

			network_data.frequency_MHz = 0;

			network_data.rssi = 0;
		}
		else 
		{

			frequency = network.frequencyMHz;
			snprintf(bssid, JSON_BUFFER_SIZE, "%02x:%02x:%02x:%02x:%02x:%02x",
				network.bssid[0], network.bssid[1], network.bssid[2], 
				network.bssid[3], network.bssid[4], network.bssid[5]);

			if ((strncmp(ssid, (char*)&network.ssid, network.ssidLength)!=0) || !networkConfigSent) {
				
				memset(ssid, 0, 128);
				strncpy(ssid, network.ssid, network.ssidLength);
				Log_Debug("SSID: %s\n", ssid);
				Log_Debug("Frequency: %dMHz\n", frequency);
				Log_Debug("bssid: %s\n", bssid);
				networkConfigSent = true;

#if (defined(IOT_CENTRAL_APPLICATION) || defined(IOT_HUB_APPLICATION))
				// Note that we send up this data to Azure if it changes, but the IoT Central Properties elements only 
				// show the data that was currenet when the device first connected to Azure.
				checkAndUpdateDeviceTwin("ssid", &ssid, TYPE_STRING, false);
				checkAndUpdateDeviceTwin("freq", &frequency, TYPE_INT, false);
				checkAndUpdateDeviceTwin("bssid", &bssid, TYPE_STRING, false);
#endif 
			}			

			memset(network_data.SSID, 0, WIFICONFIG_SSID_MAX_LENGTH);
			if (network.ssidLength <= SSID_MAX_LEGTH)
			{
				strncpy(network_data.SSID, network.ssid, network.ssidLength);
			}
			else
			{
				strncpy(network_data.SSID, network.ssid, SSID_MAX_LEGTH);
			}

			network_data.frequency_MHz = network.frequencyMHz;

			network_data.rssi = network.signalRssi;
		}	   		 	  	  	   	
#if (defined(IOT_CENTRAL_APPLICATION) || defined(IOT_HUB_APPLICATION))
		if (iothubClientHandle != NULL && !versionStringSent) {

			checkAndUpdateDeviceTwin("versionString", argv[1], TYPE_STRING, false);
			versionStringSent = true;
		}

		// AzureIoT_DoPeriodicTasks() needs to be called frequently in order to keep active
		// the flow of data with the Azure IoT Hub
		AzureIoT_DoPeriodicTasks();
#endif
    }

    ClosePeripheralsAndHandlers();
    Log_Debug("Application exiting.\n");
    return 0;
}
