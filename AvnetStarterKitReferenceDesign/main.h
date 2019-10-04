/******************************************************************************

 @file  main.h

 @brief

 Target Device: Avnet Azure Sphere Starter Kit

 Maker/Author - Markel T. Robregado
 Date: 10/1/2019
 Modification Details:
 *****************************************************************************/
#ifndef MAIN_H
#define MAIN_H

/*********************************************************************
 * CONSTANTS
 */
#define MB_SLAVEADDRESS              0x01

#define MB_ILLEGALFUNCTION           0x01 // Modbus protocol illegal function exception.
#define MB_ILLEGALDATADDRESS         0x02 // Modbus protocol illegal data address exception.
#define MB_ILLEGALDATAVALUE          0x03 // Modbus protocol illegal data value exception.
#define MB_SLAVEDEVICEFAILURE        0x04 // Modbus protocol slave device failure exception.

#define MB_SUCCESS                   0x00 // ModbusMaster success.
#define MB_INVALIDSLAVEID            0xE0 // ModbusMaster invalid response slave ID exception.
#define MB_INVALIDFUNCTION           0xE1 // ModbusMaster invalid response function exception.
#define MB_RESPONSETIMEOUT           0xE2 // ModbusMaster response timed out exception.
#define MB_INVALIDCRC                0xE3; //ModbusMaster invalid response CRC exception.

 // Modbus function codes for 16 bit access
#define MB_READHOLDINGREGISTERS      0x03 // Modbus function 0x03 Read Holding Registers DPS SUPPORTED
#define MB_WRITESINGLEREGISTER       0x06 // Modbus function 0x06 Write Single Register DPS SUPPORTED
#define MB_WRITEMULTIPLEREGISTERS    0x10 // Modbus function 0x10 Write Multiple Registers DPS SUPPORTED

#define  MAXBUFFERSIZE               64   // size of response/transmit buffers

/*********************************************************************
 * GLOBAL VARIABLES
 */

extern int fanenableGpioFd;

extern volatile bool ModbusTransactionFlag;
extern uint16_t _u16WriteAddress;                                   // slave register to which to write
extern uint16_t _u16TransmitBuffer[MAXBUFFERSIZE];         // buffer containing data to transmit to Modbus slave; set via SetTransmitBuffer()

/*********************************************************************
 * FUNCTIONS
 */
extern uint8_t ModbusMaster_writeSingleRegister(uint16_t u16WriteAddress, uint16_t u16WriteValue);

#endif /* MAIN_H */
