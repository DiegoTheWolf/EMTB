/*
BaseFile 				Copyright 2015 - 2017	Andreas Chaitidis Andreas.Chaitidis@gmail.com
Personal Rework Copyright 2017 				DiegoTheWolf diego.wolfhound@gmail.com

This program is free software : you can redistribute it and / or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef _VESCUART_h
#define _VESCUART_h

#define SERIALIO Serial
//#define DEBUGSERIAL Serial1
#include "arduino.h"

 
#include "datatypes.h"
#include "local_datatypes.h"


///PackSendPayload Packs the payload and sends it over Serial.
///Uses the Serial defined above.
///@param: payload as the payload [unit8_t Array] with length of int lenPayload
///@return the number of bytes send
int PackSendPayload(uint8_t* payload, int lenPay);

///ReceiveUartMessage receives the a message over Serial
///Define in a Config.h a SERIAL with the Serial in Arduino Style you want to you
///@parm the payload as the payload [unit8_t Array]
///@return the number of bytes receeived within the payload
int ReceiveUartMessage(uint8_t* payloadReceived);

///Help Function to print struct bldcMeasure over Serial for Debug
///Define in a Config.h the DEBUGSERIAL you want to use
void SerialPrint(const struct bldcMeasure& values);

///Help Function to print uint8_t array over Serial for Debug
///Define in a Config.h the DEBUGSERIAL you want to use
void SerialPrint(uint8_t* data, int len);

///Sends a command to VESC and stores the returned data
///@param bldcMeasure struct with received data
//@return true if sucess
bool VescUartGetValue(struct bldcMeasure& values);

///Sends a command to VESC to control the duty cycle used for cruiseCTRL
///Be careful not to set this to far from the current duty cycle. The motor will rapidly accelerate.
///@param dutyCycle as float with the dutyCycle to hold. 
void VescUartSetDuty(float dutyCycle);

///Sends a command to VESC to control the motor current
///@param current as float with the current for the motor
void VescUartSetCurrent(float current);

///Sends a command to VESC to control the motor brake
///@param breakCurrent as float with the current for the brake
void VescUartSetCurrentBrake(float brakeCurrent);

/// CRC Check
bool UnpackPayload(uint8_t* message, int lenMes, uint8_t* payload, int lenPa);

/// Takes the values from the package and wraps them in the bldcMeasure struct
bool ProcessReadPacket(uint8_t* message, bldcMeasure& values, int len);

#endif

