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

#include <applibs/log.h>
#include <applibs/i2c.h>
#include <applibs/gpio.h>

#include "mt3620_avnet_dev.h"
#include "deviceTwin.h"
#include "azure_iot_utilities.h"
#include "parson.h"
#include "build_options.h"

#include "i2c.h"
#include "main.h"
#include "dps3005.h"


bool userLedRedIsOn = false;
bool userLedGreenIsOn = false;
bool userLedBlueIsOn = false;
bool appLedIsOn = false;
bool wifiLedIsOn = false;
bool clkBoardRelay1IsOn = true;
bool clkBoardRelay2IsOn = true;

//// OLED
//uint8_t oled_ms1[CLOUD_MSG_SIZE];
//uint8_t oled_ms2[CLOUD_MSG_SIZE];
//uint8_t oled_ms3[CLOUD_MSG_SIZE];
//uint8_t oled_ms4[CLOUD_MSG_SIZE];

// DPS3005 Power Supply

float VSet = 2.5; // Set Voltage
float ASet = 0.8; // Set Current
bool On = false; // Power switch on/off
int BLed = 4; // LCD Brightness Level 
float TempThreshold = 40.0; //

extern int userLedRedFd;
extern int userLedGreenFd;
extern int userLedBlueFd;

extern int appLedFd;
extern int wifiLedFd;
extern int clickSocket1Relay1Fd;
extern int clickSocket1Relay2Fd;



extern volatile sig_atomic_t terminationRequired;

static const char cstrDeviceTwinJsonInteger[] = "{\"%s\": %d}";
static const char cstrDeviceTwinJsonFloat[] = "{\"%s\": %.2f}";
static const char cstrDeviceTwinJsonBool[] = "{\"%s\": %s}";
static const char cstrDeviceTwinJsonString[] = "{\"%s\": \"%s\"}";
#ifdef IOT_CENTRAL_APPLICATION
static const char cstrDeviceTwinJsonFloatIOTC[] = "{\"%s\": {\"value\": %.2f, \"status\" : \"completed\" , \"desiredVersion\" : %d }}";
static const char cstrDeviceTwinJsonBoolIOTC[] = "{\"%s\": {\"value\": %s, \"status\" : \"completed\" , \"desiredVersion\" : %d }}";
static const char cstrDeviceTwinJsonIntegerIOTC[] = "{\"%s\": {\"value\": %d, \"status\" : \"completed\" , \"desiredVersion\" : %d }}";
//static const char cstrDeviceTwinJsonStringIOTC[]  = "{\"%s\": {\"value\": %s, \"status\" : \"completed\" , \"desiredVersion\" : %d }}";
static const char cstrDeviceTwinJsonStringIOTC[]  = "{\"%s\": {\"value\": \"%s\", \"status\" : \"completed\" , \"desiredVersion\" : %d }}";
#endif 

static int desiredVersion = 0;

// Define each device twin key that we plan to catch, process, and send reported property for.
// .twinKey - The JSON Key piece of the key: value pair
// .twinVar - The address of the application variable keep this key: value pair data
// .twinFD - The associated File Descriptor for this item.  This is usually a GPIO FD.  NULL if NA.
// .twinGPIO - The associted GPIO number for this item.  NO_GPIO_ASSOCIATED_WITH_TWIN if NA
// .twinType - The data type for this item, TYPE_BOOL, TYPE_STRING, TYPE_INT, or TYPE_FLOAT
// .active_high - true if GPIO item is active high, false if active low.  This is used to init the GPIO 
twin_t twinArray[] = {
	//{.twinKey = "userLedRed",.twinVar = &userLedRedIsOn,.twinFd = &userLedRedFd,.twinGPIO = MT3620_RDB_LED1_RED,.twinType = TYPE_BOOL,.active_high = false},
	//{.twinKey = "userLedGreen",.twinVar = &userLedGreenIsOn,.twinFd = &userLedGreenFd,.twinGPIO = MT3620_RDB_LED1_GREEN,.twinType = TYPE_BOOL,.active_high = false},
	//{.twinKey = "userLedBlue",.twinVar = &userLedBlueIsOn,.twinFd = &userLedBlueFd,.twinGPIO = MT3620_RDB_LED1_BLUE,.twinType = TYPE_BOOL,.active_high = false},
	//{.twinKey = "appLed",.twinVar = &appLedIsOn,.twinFd = &appLedFd,.twinGPIO = AVT_LED_APP,.twinType = TYPE_BOOL,.active_high = false},
	//{.twinKey = "wifiLed",.twinVar = &wifiLedIsOn,.twinFd = &wifiLedFd,.twinGPIO = AVT_LED_WIFI,.twinType = TYPE_BOOL,.active_high = false},
	{.twinKey = "vset",.twinVar = &VSet,.twinFd = NULL,.twinGPIO = NO_GPIO_ASSOCIATED_WITH_TWIN,.twinType = TYPE_FLOAT,.active_high = false},
	{.twinKey = "aset",.twinVar = &ASet,.twinFd = NULL,.twinGPIO = NO_GPIO_ASSOCIATED_WITH_TWIN,.twinType = TYPE_FLOAT,.active_high = false},
	{.twinKey = "on",.twinVar = &On,.twinFd = NULL,.twinGPIO = NO_GPIO_ASSOCIATED_WITH_TWIN,.twinType = TYPE_BOOL,.active_high = false},
	{.twinKey = "bled",.twinVar = &BLed,.twinFd = NULL,.twinGPIO = NO_GPIO_ASSOCIATED_WITH_TWIN,.twinType = TYPE_INT,.active_high = false},
	{.twinKey = "tempthreshold",.twinVar = &TempThreshold,.twinFd = NULL,.twinGPIO = NO_GPIO_ASSOCIATED_WITH_TWIN,.twinType = TYPE_FLOAT,.active_high = false},
	//{.twinKey = "OledDisplayMsg1",.twinVar = oled_ms1,.twinFd = NULL,.twinGPIO = NO_GPIO_ASSOCIATED_WITH_TWIN,.twinType = TYPE_STRING,.active_high = true},
	//{.twinKey = "OledDisplayMsg2",.twinVar = oled_ms2,.twinFd = NULL,.twinGPIO = NO_GPIO_ASSOCIATED_WITH_TWIN,.twinType = TYPE_STRING,.active_high = true},
	//{.twinKey = "OledDisplayMsg3",.twinVar = oled_ms3,.twinFd = NULL,.twinGPIO = NO_GPIO_ASSOCIATED_WITH_TWIN,.twinType = TYPE_STRING,.active_high = true},
	//{.twinKey = "OledDisplayMsg4",.twinVar = oled_ms4,.twinFd = NULL,.twinGPIO = NO_GPIO_ASSOCIATED_WITH_TWIN,.twinType = TYPE_STRING,.active_high = true}
};

// Calculate how many twin_t items are in the array.  We use this to iterate through the structure.
int twinArraySize = sizeof(twinArray) / sizeof(twin_t);

///<summary>
///		check to see if any of the device twin properties have been updated.  If so, send up the current data.
///</summary>
void checkAndUpdateDeviceTwin(char* property, void* value, data_type_t type, bool ioTCentralFormat)
{
	int nJsonLength = -1;

	char *pjsonBuffer = (char *)malloc(JSON_BUFFER_SIZE);
	if (pjsonBuffer == NULL) {
		Log_Debug("ERROR: not enough memory to report device twin changes.");
	}

	if (property != NULL) {

		// report current device twin data as reported properties to IoTHub

		switch (type) {
		case TYPE_BOOL:
#ifdef IOT_CENTRAL_APPLICATION
			if (ioTCentralFormat) {
				nJsonLength = snprintf(pjsonBuffer, JSON_BUFFER_SIZE, cstrDeviceTwinJsonBoolIOTC, property, *(bool*)value ? "true" : "false", desiredVersion);
			}
			else
#endif 
				nJsonLength = snprintf(pjsonBuffer, JSON_BUFFER_SIZE, cstrDeviceTwinJsonBool, property, *(bool*)value ? "true" : "false", desiredVersion);

			break;
		case TYPE_FLOAT:
#ifdef IOT_CENTRAL_APPLICATION			
			if (ioTCentralFormat) {
				nJsonLength = snprintf(pjsonBuffer, JSON_BUFFER_SIZE, cstrDeviceTwinJsonFloatIOTC, property, *(float*)value, desiredVersion);
			}
			else
#endif 
				nJsonLength = snprintf(pjsonBuffer, JSON_BUFFER_SIZE, cstrDeviceTwinJsonFloat, property, *(float*)value, desiredVersion);
			break;
		case TYPE_INT:
#ifdef IOT_CENTRAL_APPLICATION		
			if (ioTCentralFormat) {
				nJsonLength = snprintf(pjsonBuffer, JSON_BUFFER_SIZE, cstrDeviceTwinJsonIntegerIOTC, property, *(int*)value, desiredVersion);
			}
			else
#endif
				nJsonLength = snprintf(pjsonBuffer, JSON_BUFFER_SIZE, cstrDeviceTwinJsonInteger, property, *(int*)value, desiredVersion);
			break;
		case TYPE_STRING:
#ifdef IOT_CENTRAL_APPLICATION			
			if (ioTCentralFormat) {
				nJsonLength = snprintf(pjsonBuffer, JSON_BUFFER_SIZE, cstrDeviceTwinJsonStringIOTC, property, (char*)value, desiredVersion);
			}
			else
#endif 
				nJsonLength = snprintf(pjsonBuffer, JSON_BUFFER_SIZE, cstrDeviceTwinJsonString, property, (char*)value, desiredVersion);
			break;
		}

		if (nJsonLength > 0) {
			Log_Debug("[MCU] Updating device twin: %s\n", pjsonBuffer);
			AzureIoT_TwinReportStateJson(pjsonBuffer, (size_t)nJsonLength);
		}
		free(pjsonBuffer);
	}
}

///<summary>
///		Parses received desired property changes.
///</summary>
///<param name="desiredProperties">Address of desired properties JSON_Object</param>
void deviceTwinChangedHandler(JSON_Object * desiredProperties)
{
	int result = 0;

	// Pull the twin version out of the message.  We use this value when we echo the new setting back to IoT Connect.
	if (json_object_has_value(desiredProperties, "$version") != 0)
	{
		desiredVersion = (int)json_object_get_number(desiredProperties, "$version");
	}

#ifdef IOT_CENTRAL_APPLICATION		

	for (int i = 0; i < (sizeof(twinArray) / sizeof(twin_t)); i++) {

		if (json_object_has_value(desiredProperties, twinArray[i].twinKey) != 0)
		{

			JSON_Object *currentJSONProperties = json_object_dotget_object(desiredProperties, twinArray[i].twinKey);

			switch (twinArray[i].twinType) {
			case TYPE_BOOL:
				*(bool*)twinArray[i].twinVar = (bool)json_object_get_boolean(currentJSONProperties, "value");
				Log_Debug("Received device update. New %s is %s\n", twinArray[i].twinKey, *(bool*)twinArray[i].twinVar ? "true" : "false");
				
				if (twinArray[i].twinKey == "on")
				{
					if (*(bool*)twinArray[i].twinVar == true)
					{
						ModbusMaster_writeSingleRegister(DPS_ONOFF, 1);
					}
					else
					{
						ModbusMaster_writeSingleRegister(DPS_ONOFF, 0);
					}
				}				
				
				/*
				result = GPIO_SetValue(*twinArray[i].twinFd, twinArray[i].active_high ? (GPIO_Value)*(bool*)twinArray[i].twinVar : !(GPIO_Value)*(bool*)twinArray[i].twinVar);

				if (result != 0) {
					Log_Debug("Fd: %d\n", twinArray[i].twinFd);
					Log_Debug("FAILURE: Could not set GPIO_%d, %d output value %d: %s (%d).\n", twinArray[i].twinGPIO, twinArray[i].twinFd, (GPIO_Value)*(bool*)twinArray[i].twinVar, strerror(errno), errno);
					terminationRequired = true;
				}
				*/
								
				checkAndUpdateDeviceTwin(twinArray[i].twinKey, twinArray[i].twinVar, TYPE_BOOL, true);
				break;
			case TYPE_FLOAT:
				*(float*)twinArray[i].twinVar = (float)json_object_get_number(currentJSONProperties, "value");
				Log_Debug("Received device update. New %s is %0.2f\n", twinArray[i].twinKey, *(float*)twinArray[i].twinVar);
				checkAndUpdateDeviceTwin(twinArray[i].twinKey, twinArray[i].twinVar, TYPE_FLOAT, true);

				if (twinArray[i].twinKey == "vset")
				{
					ModbusTransactionFlag = true;					
					ModbusMaster_writeSingleRegister(DPS_VSET, (uint16_t)((*(float*)twinArray[i].twinVar) * 100));
				}

				if (twinArray[i].twinKey == "aset")
				{
					ModbusTransactionFlag = true;
					ModbusMaster_writeSingleRegister(DPS_ISET, (uint16_t)((*(float*)twinArray[i].twinVar) * 1000));
				}

				if (twinArray[i].twinKey == "tempthreshold")
				{
					if (lsm6dsoTemperature_degC > TempThreshold)
					{
						Log_Debug("\n[Info] Temperature %.2f C Above Threshold ! ! !\n", lsm6dsoTemperature_degC);
						GPIO_SetValue(fanenableGpioFd, GPIO_Value_High);
					}
					else
					{
						GPIO_SetValue(fanenableGpioFd, GPIO_Value_Low);
					}
					
				}
		
				break;
			case TYPE_INT:
				*(int*)twinArray[i].twinVar = (int)json_object_get_number(currentJSONProperties, "value");
				Log_Debug("Received device update. New %s is %d\n", twinArray[i].twinKey, *(int*)twinArray[i].twinVar);
				checkAndUpdateDeviceTwin(twinArray[i].twinKey, twinArray[i].twinVar, TYPE_INT, true);

				if (twinArray[i].twinKey == "bled")
				{
					ModbusMaster_writeSingleRegister(DPS_B_LED, (uint16_t)(*(int*)twinArray[i].twinVar));
				}
				break;

			case TYPE_STRING:
				Log_Debug(">>> STRING!");
				strcpy((char*)twinArray[i].twinVar, json_object_get_string(currentJSONProperties, "value"));
				Log_Debug("Received device update. New %s is %s\n", twinArray[i].twinKey, (char*)twinArray[i].twinVar);
				checkAndUpdateDeviceTwin(twinArray[i].twinKey, twinArray[i].twinVar, TYPE_STRING, true);
				break;
			}
		}
	}
#else // !IOT_CENTRAL_APPLICATION		

	for (int i = 0; i < (sizeof(twinArray) / sizeof(twin_t)); i++) {

		if (json_object_has_value(desiredProperties, twinArray[i].twinKey) != 0)
		{

			switch (twinArray[i].twinType) {
			case TYPE_BOOL:
				*(bool*)twinArray[i].twinVar = (bool)json_object_get_boolean(desiredProperties, twinArray[i].twinKey);
				result = GPIO_SetValue(*twinArray[i].twinFd, twinArray[i].active_high ? (GPIO_Value)*(bool*)twinArray[i].twinVar : !(GPIO_Value)*(bool*)twinArray[i].twinVar);

				if (result != 0) {
					Log_Debug("Fd: %d\n", twinArray[i].twinFd);
					Log_Debug("FAILURE: Could not set GPIO_%d, %d output value %d: %s (%d).\n", twinArray[i].twinGPIO, twinArray[i].twinFd, (GPIO_Value)*(bool*)twinArray[i].twinVar, strerror(errno), errno);
					terminationRequired = true;
				}
				Log_Debug("Received device update. New %s is %s\n", twinArray[i].twinKey, *(bool*)twinArray[i].twinVar ? "true" : "false");
				checkAndUpdateDeviceTwin(twinArray[i].twinKey, twinArray[i].twinVar, TYPE_BOOL, true);
				break;
			case TYPE_FLOAT:
				*(float*)twinArray[i].twinVar = (float)json_object_get_number(desiredProperties, twinArray[i].twinKey);
				Log_Debug("Received device update. New %s is %0.2f\n", twinArray[i].twinKey, *(float*)twinArray[i].twinVar);
				checkAndUpdateDeviceTwin(twinArray[i].twinKey, twinArray[i].twinVar, TYPE_FLOAT, true);
				break;
			case TYPE_INT:
				*(int*)twinArray[i].twinVar = (int)json_object_get_number(desiredProperties, twinArray[i].twinKey);
				Log_Debug("Received device update. New %s is %d\n", twinArray[i].twinKey, *(int*)twinArray[i].twinVar);
				checkAndUpdateDeviceTwin(twinArray[i].twinKey, twinArray[i].twinVar, TYPE_INT, true);
				break;

			case TYPE_STRING:
				strcpy((char*)twinArray[i].twinVar, (char*)json_object_get_string(desiredProperties, twinArray[i].twinKey));
				Log_Debug("Received device update. New %s is %s\n", twinArray[i].twinKey, (char*)twinArray[i].twinVar);
				checkAndUpdateDeviceTwin(twinArray[i].twinKey, twinArray[i].twinVar, TYPE_STRING, true);
				break;
			}
		}
	}
#endif 

}
