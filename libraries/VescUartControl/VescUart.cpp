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

#include "VescUart.h"
#include "buffer.h"
#include "crc.h"



uint32_t timeLastRequest; //Time
const uint8_t waitForMsg = 100; //[ms]

int ReceiveUartMessage(uint8_t* payloadReceived) {

	//Messages <= 255 start with 2. 2nd byte is length
	//Messages >255 start with 3. 2nd and 3rd byte is length combined with 1st >>8 and then &0xFF
	 
	int counter = 0;
	int endMessage = 256;
	bool messageRead = false;
	uint8_t messageReceived[256];
	int lenPayload = 0;

	while (SERIALIO.available()) {

		messageReceived[counter++] = SERIALIO.read();

		if (counter == 2) {//case if state of 'counter' with last read 1

			switch (messageReceived[0])
			{
			case 2:
				endMessage = messageReceived[1] + 5; //Payload size + 2 for size + 3 for SRC and End.
				lenPayload = messageReceived[1];
				break;
			case 3:
				//ToDo: Add Message Handling > 255 (starting with 3)
				break;
			default:
				break;
			}

		}
		if (counter >= sizeof(messageReceived))
		{
			break;
		}

		if (counter == endMessage && messageReceived[endMessage - 1] == 3) {//+1: Because of counter++ state of 'counter' with last read = "endMessage"
			messageReceived[endMessage] = 0;
#ifdef DEBUGSERIAL
			DEBUGSERIAL.println("End of message reached!");
#endif			
			messageRead = true;
			break; //Exit if end of message is reached, even if there is still more data in buffer. 
		}
	}
	bool unpacked = false;
	if (messageRead) {
		unpacked = UnpackPayload(messageReceived, endMessage, payloadReceived, messageReceived[1]);
	}
	if (unpacked)
	{
		return lenPayload; //Message was read

	}
	else {
		return 0; //No Message Read
	}
}

bool UnpackPayload(uint8_t* message, int lenMes, uint8_t* payload, int lenPay) {
	uint16_t crcMessage = 0;
	uint16_t crcPayload = 0;
	//Rebuild src:
	crcMessage = message[lenMes - 3] << 8;
	crcMessage &= 0xFF00;
	crcMessage += message[lenMes - 2];
#ifdef DEBUGSERIAL
	DEBUGSERIAL.print("SRC received: "); DEBUGSERIAL.println(crcMessage);
#endif // DEBUG

	//Extract payload:
	memcpy(payload, &message[2], message[1]);

	crcPayload = crc16(payload, message[1]);
#ifdef DEBUGSERIAL
	DEBUGSERIAL.print("SRC calc: "); DEBUGSERIAL.println(crcPayload);
#endif
	if (crcPayload == crcMessage)
	{
#ifdef DEBUGSERIAL
		DEBUGSERIAL.print("Received: "); SerialPrint(message, lenMes); DEBUGSERIAL.println();
		DEBUGSERIAL.print("Payload :      "); SerialPrint(payload, message[1] - 1); DEBUGSERIAL.println();
#endif // DEBUG

		return true;
	}
	else
	{
		return false;
	}
}

int PackSendPayload(uint8_t* payload, int lenPay) {
	uint16_t crcPayload = crc16(payload, lenPay);
	int count = 0;
	uint8_t messageSend[256];

	if (lenPay <= 256)
	{
		messageSend[count++] = 2;
		messageSend[count++] = lenPay;
	}
	else
	{
		messageSend[count++] = 3;
		messageSend[count++] = (uint8_t)(lenPay >> 8);
		messageSend[count++] = (uint8_t)(lenPay & 0xFF);
	}
	memcpy(&messageSend[count], payload, lenPay);

	count += lenPay;
	messageSend[count++] = (uint8_t)(crcPayload >> 8);
	messageSend[count++] = (uint8_t)(crcPayload & 0xFF);
	messageSend[count++] = 3;
	messageSend[count] = NULL;

#ifdef DEBUGSERIAL
	DEBUGSERIAL.print("UART package send: "); SerialPrint(messageSend, count);

#endif // DEBUG

	//Sending package
	SERIALIO.write(messageSend, count);


	//Returns number of send bytes
	return count;
}


bool ProcessReadPacket(uint8_t* message, bldcMeasure& values, int len) {
	COMM_PACKET_ID packetId;
	int32_t ind = 0;

	packetId = (COMM_PACKET_ID)message[0];
	message++;//Eliminates the message id
	len--;

	switch (packetId)
	{
	case COMM_GET_VALUES:
		//values.temp_mos1 = buffer_get_float16(data, 10.0, &ind);
		//values.temp_mos2 = buffer_get_float16(data, 10.0, &ind);
		//values.temp_mos3 = buffer_get_float16(data, 10.0, &ind);
		//values.temp_mos4 = buffer_get_float16(data, 10.0, &ind);
		//values.temp_mos5 = buffer_get_float16(data, 10.0, &ind);
		//values.temp_mos6 = buffer_get_float16(data, 10.0, &ind);
		//values.temp_pcb = buffer_get_float16(data, 10.0, &ind);
		ind = 14; //Skip
		values.current_motor = buffer_get_float32(message, 100.0, &ind);
		values.current_in  = buffer_get_float32(message, 100.0, &ind);
		values.duty_now  = buffer_get_float16(message, 1000.0, &ind);
		values.rpm = buffer_get_int32(message, &ind);
		values.v_in  = buffer_get_float16(message, 10.0, &ind);
		values.amp_hours = buffer_get_float32(message, 10000.0, &ind);
		values.amp_hours_charged = buffer_get_float32(message, 10000.0, &ind);
		//values.watt_hours = buffer_get_float32(data, 10000.0, &ind);
		//values.watt_hours_charged = buffer_get_float32(data, 10000.0, &ind);
		//values.tachometer = buffer_get_int32(message, &ind);
		ind += 12; //Skip 
		values.tachometerAbs = buffer_get_int32(message, &ind);
		return true;
		break;

	default:
		return false;
		break;
	}

}

bool VescUartRequestValues() {
	if (!SERIALIO.available() && millis() - timeLastRequest > waitForMsg ) { // Only write if nothings is there to read
		uint8_t command[1] = { COMM_GET_VALUES };
		PackSendPayload(command, 1);
		timeLastRequest = millis();
	}
}

bool VescUartGetValue(bldcMeasure& values) {
	uint8_t payload[256];
	if (SERIALIO.available()){
		int lenPayload = ReceiveUartMessage(payload);
		if (lenPayload > 55) {
			bool read = ProcessReadPacket(payload, values, lenPayload); //returns true if sucessful
			return read;
		}
		else
		{
			return false;
		}
	} else {
		return false;
	}
}

void VescUartSetDuty(float dutyCycle) {
	int32_t index = 0;
	uint8_t payload[5];
		
	payload[index++] = COMM_SET_DUTY ;
	buffer_append_int32(payload, (int32_t)(dutyCycle * 100000), &index);
	PackSendPayload(payload, 5);
}

void VescUartSetCurrent(float current) {
	int32_t index = 0;
	uint8_t payload[5];
		
	payload[index++] = COMM_SET_CURRENT ;
	buffer_append_int32(payload, (int32_t)(current * 1000), &index);
	PackSendPayload(payload, 5);
}

void VescUartSetCurrentBrake(float brakeCurrent) {
	int32_t index = 0;
	uint8_t payload[5];

	payload[index++] = COMM_SET_CURRENT_BRAKE;
	buffer_append_int32(payload, (int32_t)(brakeCurrent * 1000), &index);
	PackSendPayload(payload, 5);

}

void SerialPrint(uint8_t* data, int len) {

	//	DEBUGSERIAL.print("Data to display: "); DEBUGSERIAL.println(sizeof(data));
#ifdef DEBUGSERIAL
	for (int i = 0; i <= len; i++)
	{
		DEBUGSERIAL.print(data[i]);
		DEBUGSERIAL.print("\t");
	}
	DEBUGSERIAL.println("");
#endif
}


void SerialPrint(const bldcMeasure& values) {
#ifdef DEBUGSERIAL
	DEBUGSERIAL.print("current_motor: "); DEBUGSERIAL.println(values.current_motor);
	DEBUGSERIAL.print("current_in: "); DEBUGSERIAL.println(values.current_in);
	DEBUGSERIAL.print("duty_now: "); DEBUGSERIAL.println(values.duty_now);
	DEBUGSERIAL.print("rpm: "); DEBUGSERIAL.println(values.rpm);
	DEBUGSERIAL.print("inputVoltage: "); DEBUGSERIAL.println(values.v_in);
	DEBUGSERIAL.print("amp_hours: "); DEBUGSERIAL.println(values.amp_hours);
	DEBUGSERIAL.print("ampHoursCharges: "); DEBUGSERIAL.println(values.amp_hours_charged);
	DEBUGSERIAL.print("tachometerAbs: "); DEBUGSERIAL.println(values.tachometerAbs);
#endif
}

