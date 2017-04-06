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

/* PIN Definitions -> ProMini
  DTR|TX0|RXI|VCC|GND|GND --> FTDI programmer

										TX		RAW
										RX		GND --> Step-Down GND
                    RST		RST
                    GND		VCC --> Step-Down +3.3V
                        A5
                    2			A3 
                        A4
                    3			A2 
    CE nRF24L01 <-- 4			A1 
    CS nRF24L01 <--	5			A0 
                    6			13 --> nRF24L01 SCK  
                    7			12 --> nRF24L01 MISO 
                    8			11 --> nRF24L01 MOSI 
        LED FWD <--	9			10 --> LED BACK
*/

#define PIN_LED_FWD 9
#define PIN_LED_BACK 10

const uint8_t channel = 77;
const uint64_t pipe = 0x52582d5458; // 'RX-TX' pipe
const uint8_t timeout = 100;        // [ms]
const uint8_t deadband = 255;       // no throttle can be given, only when overwritten bei the Remote data
const uint8_t amp_fwd = 0;          // Turn off output
const uint8_t amp_break = 0;        // Turn off output
const uint16_t waitForSend = 2000;  // [ms]
// ToDo: Safety: Start Writing to VESC only after Throttle was centered.

uint32_t timeLastRemote;
uint8_t lastDuty;
uint32_t lastLED;
bool LEDstate; //bit0 --> 1=BREAK 0=FWD // bit1 --> 1=ON 0=OFF
bool startSendingToVESC = false;
uint32_t timeWaiting;
uint32_t _millis;

struct RemoteData RemoteDataStruct;

struct bldcMeasure VescMeasuredValues;

const struct LEDdata {
	const uint8_t count;
	const uint32_t fwd_on=200;
	const uint32_t fwd_off=500;
	const uint32_t break_on=500;
	const uint32_t break_off=100;
}LED_const;

// Set up nRF24L01 radio on SPI bus plus pins 7 & 8 (CE & CS)
RF24 radio(7, 8);

// Define the array of leds
CRGB led_fwd[NUM_LEDS];
CRGB led_back[NUM_LEDS];

void setup() {
	
	FastLED.addLeds<WS2812B, PIN_LED_FWD, RGB>(led_fwd, LED_const.count);
	FastLED.addLeds<WS2812B, PIN_LED_BACK, RGB>(led_back, LED_const.count);
	
	fill_solid(led_fwd, LED_const.count, CRGB( 0, 50, 0)); //slight green
	fill_solid(led_back, LED_const.count, CRGB( 0, 0, 50)); //slight blue
	FastLED.show();
	
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
	
	fill_solid(led_fwd, LED_const.count, CRGB(230, 230, 255)); //white (blueish) front
	fill_solid(led_back, LED_const.count, CRGB(150, 0, 0)); //red rear
	FastLED.show();
}

void loop() {
  bool gotMsg;

  // Get values from VESC
  // Fill FIFO with AckPayload, for next return
  if (VescUartGetValue(VescMeasuredValues) && radio.available()) {
    radio.writeAckPayload(pipe, &VescMeasuredValues, sizeof(VescMeasuredValues));
  }
	
  // Read Data from TX
	while (radio.available()) { // Read everything
		gotMsg = radio.read(&RemoteData, sizeof(RemoteData));
		timeLastRemote = millis();
	}
  if (!gotMsg) {
    // If no data fetched and timeout reached set values to center/default.
    if ((millis() - timeLastRemote) > timeout) {
      RemoteData.thr = 0;
      RemoteData.cruise = false;
    }
  }

	if(startSendingToVESC){
		// Set Duty once if over 5%, else reset duty and apply current
		if (RemoteData.cruise) {
			if (lastDuty == 0 && VescMeasuredValues.duty_now > 5) {
				lastDuty = VescMeasuredValues.duty_now;
				VescUartSetDuty(lastDuty); 
			}
		} else {
			// Set VESC currents && reset lastDuty
			lastDuty = 0;
			LEDstate &= 0b10;
			if (RemoteData.thr > deadband) {
				VescUartSetCurrent(RemoteData.thr * (amp_fwd / 2.0) / 127.0);
				
			} else if (RemoteData.thr < -deadband) {
				VescUartSetCurrentBrake(RemoteData.thr * (amp_break / 5.0) / -127.0);
				LEDstate |= 0b01
			} else {
				VescUartSetCurrent(0);
				VescUartSetCurrentBrake(0);
			}
		}
	} else {
		_millis = millis();
		if (Abs(RemoteData.thr) - deadband < 10) {
			if (timeWaiting == 0){
				timeWaiting = _millis;
			}
			if (_millis - timeWaiting > waitForSend){
				startSendingToVESC = true;
			}
		} else {
			timeWaiting = 0;
		}
	}
	
	//LED
	uint32_t compTime;
	if(LEDstate & 1 << 1) { //if ON
		for (int i = 2; i < 12; i++){
			led_back[i] = CRGB(150, 0, 0);
		}
		if(LEDstate & 1){ //BREAK
			compTime = LED_const.break_on;
			led_back[0] = CRGB::RED;
			led_back[1] = CRGB::RED;
			led_back[12] = CRGB::RED;
			led_back[13] = CRGB::RED;
		} else { // FWD
			compTime = LED_const.fwd_on;
			led_back[0] = CRGB(150, 0, 0);
			led_back[1] = CRGB(150, 0, 0);
			led_back[12] = CRGB(150, 0, 0);
			led_back[13] = CRGB(150, 0, 0);
		}
	} else { //if OFF
		led_back.clear();
		if(LEDstate & 1){ //BREAK
			compTime = LED_const.break_off;
		} else { //FWD
			compTime = LED_const.fwd_off;
		}
	}
	
	_millis = millis();
	if (_millis - lastLED > compTime)
	{
		FastLED.show();
		lastLED = _millis;
	}
}
