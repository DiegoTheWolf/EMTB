#include <Arduino.h>
#include <RF24.h> //<SPI.h> included
#include <RF24_config.h>
#include <VescUart.h>        //VESC
#include <buffer.h>          //VESC
#include <crc.h>             //VESC
#include <datatypes.h>       //VESC
#include <local_datatypes.h> //VESC
#include <nRF24L01.h>
#include <printf.h>

const uint8_t channel = 77;
const uint64_t pipe = 0x52582d5458; // 'RX-TX' pipe
const uint16_t looptime = 20000;    // [µs] 20ms = 50Hz
const uint8_t timeout = 100;        // [ms]
const uint8_t deadband = 255;       // no throttle can be given, only when overwritten bei the Remote data
const uint8_t amp_fwd = 0;          // Turn off output
const uint8_t amp_break = 0;        // Turn off output
// ToDo: Read Values form EEPROM
// ToDo: MenuMode from TX at Startup (Cruis + Full Breaks)
// ToDo: Safety: Start Writing to VESC only after Throttle was centered.

uint32_t timeLastRemote;

struct RemoteDataStruct {
  int8_t thr;
  bool cruise;
  uint8_t deadband;
  uint8_t _amp_fwd;
  uint8_t _amp_break;
} RemoteData;

struct bldcMeasure VescMeasuredValues;
/*
struct bldcMeasure {
        // 7 Values int16_t not read(14 byte)
        float avgMotorCurrent;
        float avgInputCurrent;
        float dutyCycleNow;
        long rpm;
        float inpVoltage;
        float ampHours;
        float ampHoursCharged;
        // 2 values int32_t not read (8 byte)
        long tachometer;
        long tachometerAbs;
}
*/

// Set up nRF24L01 radio on SPI bus plus pins 7 & 8 (CE & CS)
RF24 radio(7, 8);

void setup() {
  // Setup UART port
  Serial.begin(115200);

  // Setup and configure rf radio
  radio.begin();

  radio.setChannel(channel);
  radio.setPALevel(RF24_PA_MIN); // RF24_PA_MIN=-18dBm, RF24_PA_LOW=-12dBm, RF24_PA_MED=-6dBM, and RF24_PA_HIGH=0dBm.
  radio.setDataRate(RF24_2MBPS); // RF24_250KBPS for 250kbs, RF24_1MBPS for 1Mbps, or RF24_2MBPS for 2Mbps
  radio.enableDynamicPayloads(); // enabled for 'enableAckPayload()
  radio.enableAckPayload();

  radio.setRetries(1, 15);         // delay (n-1)x250µs // # retries max 15
  radio.setCRCLength(RF24_CRC_16); // Use 16-bit CRC for safety

  radio.openReadingPipe(1, pipe);

  radio.startListening(); // Start listening
  radio.printDetails();   // Dump the configuration for debugging
  radio.powerUp();        // Leave low-power mode - making radio more responsive. // powerDown() for low-power
}

bool ReadWIFIData() {
  while (radio.available()) { // Read everything
    radio.read(&RemoteData, sizeof(RemoteData));
    timeLastRemote = millis();
    return true;
  }
  return false;
}

void loop() {
  bool gotMsg;
  // Get values from VESC
  if (VescUartGetValue(VescMeasuredValues)) {
    gotMsg = true;
  } else {
    gotMsg = false;
  }

  // Fill FIFO with AckPayload, for next return
  if (gotMsg) {
    radio.writeAckPayload(pipe, &VescMeasuredValues, sizeof(VescMeasuredValues));
  }

  // Read Data from TX
  if (!ReadWIFIData()) {
    // If no data fetched and timeout reached set values to center/default.
    if ((millis() - timeLastRemote) > timeout) {
      RemoteData.thr = 0;
      RemoteData.cruise = false;
    }
  }

  // Set VESC values
  if (RemoteData.thr > deadband) {
    VescUartSetCurrent(RemoteData.thr * amp_fwd / 127.0);
  } else if (RemoteData.thr < -deadband) {
    VescUartSetCurrentBrake((RemoteData.thr + 127.0) * amp_break / (127.0 - amp_break));
  } else {
    VescUartSetCurrent(0);
    VescUartSetCurrentBrake(0);
  }

  // managing looptime
  // On overflow the next loop runs directly. This is acceptable.
  unsigned long started_waiting_at = micros();
  while (micros() - started_waiting_at < looptime)
    ;
}
