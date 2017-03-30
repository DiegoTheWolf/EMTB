/*
 // color definitions
 TFT_BLACK       0x0000
 TFT_NAVY        0x000F
 TFT_DARKGREEN   0x03E0
 TFT_DARKCYAN    0x03EF
 TFT_MAROON      0x7800
 TFT_PURPLE      0x780F
 TFT_OLIVE       0x7BE0
 TFT_LIGHTGREY   0xC618
 TFT_DARKGREY    0x7BEF
 TFT_BLUE        0x001F
 TFT_GREEN       0x07E0
 TFT_CYAN        0x07FF
 TFT_RED         0xF800
 TFT_MAGENTA     0xF81F
 TFT_YELLOW      0xFFE0
 TFT_WHITE       0xFFFF
 TFT_ORANGE      0xFD20
 TFT_GREENYELLOW 0xAFE5
 TFT_PINK        0xF81F
*/

#include <RF24.h>
#include <RF24_config.h>
#include <SPI.h>
#include <TFT_ST7735.h>
#include <buffer.h>          //VESC
#include <crc.h>             //VESC
#include <datatypes.h>       //VESC
#include <local_datatypes.h> //VESC
#include <nRF24L01.h>
#include <printf.h>
#include <EEPROM.h>
#include <Bounce2.h>
#include <SD.h>

/* PIN Definitions -> ProMini
		DTR|TX0|RXI|VCC|GND|GND --> FTDI programmer

				VESC RX <-- TX		RAW 
				VESC TX <-- RX		GND --> Step-Down GND
										RST		RST 
										GND		VCC --> Step-Down +3.3V
													A5 --> Poti(Throttle)
	Button Cruise <-- 2			A3
													A4
Button Settings <-- 3			A2
		CE nRF24L01 <-- 4			A1
		CS nRF24L01 <--	5			A0 
			CS SDcard <-- 6			13 --> nRF24L01 SCK  | SDcard SCK	 | TFT SCK
			 	 CS TFT <-- 7			12 --> nRF24L01 MISO | SDcard MISO
		  Reset TFT <-- 8			11 --> nRF24L01 MOSI | SDcard MOSI | TFT SDA
				 A0 TFT <--	9			10 --> TFT LED
*/

//Idea: Second/Third Poti for max AMPS

#define PIN_BTN_CRUISE 2
#define PIN_BTN_SETTINGS 3
#define PIN_RADIO_CS 4
#define PIN_RADIO_CE 5
#define PIN_SDCARD_CS 6
#define PIN_TFT_CS 7
#define PIN_TFT_RESET 8
#define PIN_TFT_A0 9
#define PIN_TFT_LED 10
#define PIN_POTI A5

//constants
const uint8_t channel = 77;
const uint64_t pipe = 0x52582d5458; // 'RX-TX' pipe
const uint32_t time_settings = 4000; // [ms]
const uint32_t SdAbortOk = 4000; // [ms]
const uint8_t eeDeadband = 0;
const uint8_t eeFwd = 1;
const uint8_t eeBreak = 2;
const uint16_t TFTrefresh = 500; // [ms]
uint32_t TFTlastPaint;
const uint16_t SDrefresh = 500; // [ms]
uint32_t SDlastPrint;
const uint8_t wheelsize = 200 // [mm]
const uint8_t gearratio = 3 // [1:X]
const uint8_t pulse_rpm = 42 // Number of poles * 3
const uint8_t erpm_rpm = 7 // Number of polse / 2
const float dist_corr_factor = 0.8 // Number of polse / 2
const float ratio_RpmSpeed = (wheelsize * 3.141 * 60) / (erpm_rpm * gearratio * 1000000);  //ERPM to km/h
const float ratio_TachoDist = ((wheelsize * 3.141) / (pulse_rpm * gearratio * 1000000)) * dist_corr_factor; //pulses to km
uint32_t SetPushTime;
uint32_t ridetime;
bool SendEnabled;
bool hasSDcard;
const uint16_t waitBeforeSend = 5000; //[ms]
File logfile;

//Structs // ToDo: Outsource
struct point { 
  uint8_t X;
  uint8_t Y;
};

struct point battery = {82, 54};
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

struct RemoteDataStruct {
  int8_t thr;
  bool cruise;
  uint8_t _deadband;
  uint8_t _amp_fwd;
  uint8_t _amp_break;
} RemoteData;

// functions
void DrawBattery(uint16_t color);
void FillBattery(uint8_t value);

//objects
RF24 radio(PIN_RADIO_CS, PIN_RADIO_CE); // Set up nRF24L01 radio on SPI bus
TFT_ST7735 tft = TFT_ST7735();// pins defined in User_Setup.h // ToDo: Move pin definition to this file
Bounce DEB_cruise = Bounce(); 

// average
uint16_t avgSum = 0; 
uint8_t avgCnt = 10; 
uint8_t avg[avgCnt];
uint8_t avgIdx = 0;

void setup() {
	
	pinMode(PIN_BTN_CRUISE, INPUT_PULLUP);
	pinMode(PIN_BTN_SETTINGS, INPUT_PULLUP);
	pinMode(PIN_POTI, INPUT_PULLUP);
	
	DEB_cruise.attach(PIN_BTN_CRUISE); // standard-interval 10 ms
	
	EEPROM.get(eeDeadband, RemoteData._deadband);
	EEPROM.get(eeFwd, RemoteData._amp_fwd);
	EEPROM.get(eeBreak, RemoteData._amp_break);
	
	tft.init();
  tft.setRotation(0); // portrait
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawCentreString("START", 64, 40, 4);
	
	// Check if the button is pressed at startup.
	// Holding it down longer then "time_settings" will enter the drawSettingsMenu and abort startup
	while (digitalRead(PIN_BTN_SETTINGS)){
		tft.setTextColor(TFT_YELLOW, TFT_BLACK);
		tft.drawCentreString("Settings...", 64, 40, 2);
		if (millis() > time_settings){drawSettingsMenu();}
	}
	tft.setTextColor(TFT_WHITE, TFT_BLACK);
	tft.drawCentreString("Connecting", 64, 40, 2);
	tft.drawCentreString("SD Card", 64, 70, 2);
	if (!SD.begin(chipSelect)) {
		WaitForSdAbort();
  } else {
		hasSDcard = true;
		logfile = SD.open("logfile.txt", FILE_WRITE);
		if (!logfile) WaitForSdAbort();
	}
 
	tft.drawCentreString("Serial", 64, 70, 2);
  Serial.begin(115200);
  while (!Serial()){} //Wait for Serial

	tft.drawCentreString("Radio", 64, 70, 2);
  // Setup and configure rf radio
  radio.begin();
  radio.setChannel(channel);
  radio.setPALevel(RF24_PA_MIN); // RF24_PA_MIN=-18dBm, RF24_PA_LOW=-12dBm, RF24_PA_MED=-6dBM, and RF24_PA_HIGH=0dBm.
  radio.setDataRate(RF24_2MBPS); // RF24_250KBPS for 250kbs, RF24_1MBPS for 1Mbps, or RF24_2MBPS for 2Mbps
  radio.enableDynamicPayloads(); // enabled for 'enableAckPayload()
  radio.enableAckPayload();
  radio.setRetries(1, 15);         // delay (n-1)x250µs // #retries max 15
  radio.setCRCLength(RF24_CRC_16); // Use 16-bit CRC for safety

  radio.openWritingPipe(pipe);

  radio.powerUp(); // Leave low-power mode - making radio more responsive. // powerDown() for low-power
	
	tft.fillScreen(TFT_BLACK);
	DrawBattery(TFT_WHITE);
  FillBattery(0);
}

void WaitForSdAbort(){
	uint16_t waitStart = millis();
	bool pressStart;
	tft.drawCentreString("FAILED ...", 64, 100, 4);
	tft.drawString("Continue with [CRUISE] ...", 0, 100, 2);
	while (1){
		pressStart = false;
		while (digitalRead(PIN_BTN_CRUISE)){
			if (!pressStart){
				pressStart = true;
				waitStart = millis();
			}
			tft.setTextColor(TFT_YELLOW, TFT_BLACK);
			tft.drawCentreString("Settings...", 64, 40, 2);
			if (millis() - waitStart > SdAbortOk){
				hasSDcard = false;
				goto Abort;
			}
		}
	}
Abort:
}

void loop() {
  // read Poti
	avg[avgIdx] = analogRead(PIN_POTI);
	avgSum -= avg[avgIdx];
	avg[avgIdx] = value;
	avgSum += avg[avgIdx];
	avgIdx++;
	if (avgIdx == avgCnt) avgIdx = 0;
	RemoteData.thr = avgSum / avgCnt;
	
	//readButtons
	DEB_cruise.update();
	RemoteData.cruise = DEB_cruise.read();
	
	if (SendEnabled) {
		// send values to RX
		bool sendOK = radio.write(&RemoteData, sizeof(RemoteData));

		// recieve AckPayload
		while (radio.isAckPayloadAvailable()) {
			radio.read(&VescMeasuredValues, sizeof(VescMeasuredValues));
			bool recOK = true;
		}
	} else {
		if (millis() > waitBeforeSend) SendEnabled = true;
	}
	
	uint32_t _millis; //buffer 1x instead of 5x exec
  // Write Readings and AckPayload into a Logfile on the SD
	if (hasSDcard){
		_millis = millis();
		if (_millis > SDlastPrint + SDrefresh){
			String logString;
			logString += String(RemoteData.thr) 
								+ ";" + String(RemoteData.cruise) 
								+ ";" + String(RemoteData._deadband)
								+ ";" + String(RemoteData._amp_fwd)
								+ ";" + String(RemoteData._amp_break)
								+ ";" + String(VescMeasuredValues.avgMotorCurrent)
								+ ";" + String(VescMeasuredValues.avgInputCurrent)
								+ ";" + String(VescMeasuredValues.dutyCycleNow)
								+ ";" + String(VescMeasuredValues.rpm)
								+ ";" + String(VescMeasuredValues.inpVoltage)
								+ ";" + String(VescMeasuredValues.ampHours)
								+ ";" + String(VescMeasuredValues.ampHoursCharged)
								+ ";" + String(VescMeasuredValues.tachometer)
			dataFile.println(logString);
			SDlastPrint = _millis;
		}
	}
  // Calculate Averages

  // Write Average-Values to screen (if changed)
	_millis = millis(); //buffer 1x instead of 5x exec
	if (_millis > TFTlastPaint + TFTrefresh){
		if(digitalRead(PIN_BTN_SETTINGS)) {
			if (SetPushTime == 0) SetPushTime = _millis;
			if (_millis > SetPushTime + 2000){
				SetPushTime = 0;
				ridetime = _millis; //Reset Ridetime. This is inside the TFT Loop so it doesn't get called every loop.
			}
		}
		drawValues();
		TFTlastPaint=_millis;
	}

}

void drawLabels(){
	tft.setTextColor(TFT_WHITE, TFT_BLACK);
	tft.drawRightString("km/h", 0, 0, 4);
	tft.drawRightString("A", 0, 40, 4);
	tft.drawRightString("A", 0, 80, 4);
	tft.drawString("Mot", 0, 40, 2);
	tft.drawString("Bat", 0, 80, 2);
	tft.drawString("Time", 0, 100, 2);
	tft.drawString("Distance", 0, 120, 2);
	tft.drawString("DutyC", 0, 140, 2);
}

void drawValues(){
	tft.drawNumber(VescMeasuredValues.rpm * ratio_RpmSpeed, 0, 0, 4);
	tft.drawNumber(VescMeasuredValues.avgMotorCurrent, 0, 40, 4);
	tft.drawNumber(VescMeasuredValues.avgInputCurrent, 0, 80, 4);
	tft.drawNumber(ridetime / (1000 * 60), 0, 100, 4);
	tft.drawChar(":", 20, 40, 4);
	tft.drawNumber((ridetime / 1000) % 60, 25, 100, 4);
	tft.drawNumber(VescMeasuredValues.tachometer * ratio_TachoDist, 0, 120, 4);
	tft.drawNumber(VescMeasuredValues.dutyCycleNow, 0, 140, 4);
}

// Return is an RGB value.
uint16_t GradientRYG(uint8_t value) {
  // From green to yellow G stays at 0xFF and R goes from 0x00 to 0xFF
  // Everything over yellow has R = 0xFF
  if (value < 128) {
    return 0xF800 + ((value * 2) << 3);
  } else {
    return 0x7E0 + (((255 - value) >> 2) << 11);
  }
}

// ToDo: Comp: multiple DrawFastLine next to each other or FillRect
void DrawBattery(uint16_t color) {
  // fillRect(x, y, w, h, color);
  tft.fillRect(battery.X + 3, battery.Y + 5, 14, 3, color);   // TopLeft
  tft.fillRect(battery.X + 14, battery.Y, 3, 4, color);       // TopUpLeft
  tft.fillRect(battery.X + 17, battery.Y, 11, 2, color);      // TopMid
  tft.fillRect(battery.X + 28, battery.Y, 3, 4, color);       // TopUpRight
  tft.fillRect(battery.X + 28, battery.Y + 5, 14, 3, color);  // TopRight
  tft.fillRect(battery.X + 42, battery.Y + 5, 3, 100, color); // Right
  tft.fillRect(battery.X + 3, battery.Y + 74, 39, 3, color);  // Bottom
  tft.fillRect(battery.X, battery.Y + 5, 3, 100, color);      // Left
}

void FillBattery(uint8_t value) {
  uint8_t line = 99 + battery.Y - ((value / 8) * 3);
  tft.fillRect(battery.X + 4, battery.Y + 9, 37, 92, TFT_BLACK); // Overwrite all previous
  tft.fillRect(battery.X + 18, line - 3, 9, 5, TFT_BLACK);
  if (value > 0) {
    while (line <= 153) { // last line
      uint8_t color_input = map(line, 60, 153, 0, 255);
      if (line == 60) {
        tft.fillRect(battery.X + 18, line - 3, 9, 5, TFT_GREEN);
      } else if (line == 150) {
        tft.fillRect(battery.X + 4, line, 37, 5, TFT_RED);
      } else {
        // drawFastHLine(x, y, w, color)
        tft.drawFastHLine(battery.X + 4, line, 37, GradientRYG(color_input));
        tft.drawFastHLine(battery.X + 4, line + 1, 37, GradientRYG(color_input));
      }
      line -= 3;
    }
  } else { // draw a red X
    // drawLine(x0, y0, x1, y1, color)
    tft.drawLine(battery.X + 10, battery.Y + 36, battery.X + 44, battery.Y + 76, TFT_RED); // X+-12 Y+-20
    tft.drawLine(battery.X + 9, battery.Y + 37, battery.X + 43, battery.Y + 77, TFT_RED);
    tft.drawLine(battery.X + 11, battery.Y + 35, battery.X + 45, battery.Y + 75, TFT_RED);
    tft.drawLine(battery.X + 44, battery.Y + 36, battery.X + 10, battery.Y + 76, TFT_RED); // other diagonal
    tft.drawLine(battery.X + 43, battery.Y + 37, battery.X + 9, battery.Y + 77, TFT_RED);
    tft.drawLine(battery.X + 45, battery.Y + 35, battery.X + 11, battery.Y + 75, TFT_RED);
  }
}

//Enter Settings Mode. Exit only over reset.
void drawSettingsMenu(){
	drawSettings();
	drawSettingValues();
	uint8_t currentSetting = 0; // 0=save // 1=deadband // 2=amp_fwd // 3=amp_break
	bool ok;
	bool trigger;
	uint16_t stick;
	while(1){
		ok = digitalRead(PIN_BTN_CRUISE);
		stick = analogRead(PIN_POTI);
		//some movement has to me done to trigger
		if (stick > 712 && !trigger){ //mid 512
			currentSetting++;
			trigger = true;
		} else if (stick < 312 && !trigger) {
			currentSetting--;
			trigger = true;
		} else if (stick < 612 && stick > 412 && trigger){ //100 difference so it won't jitter
			trigger = false;
		}
	}
}

void drawSettings(){
	tft.setTextSize(1); // no scaling
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
	tft.drawString("SETTINGS", 0, 0, 4); //Font 4
	
	tft.setTextColor(TFT_WHITE, TFT_BLACK);
	tft.drawString("Deadband", 0, 30, 2);
	tft.drawString("FWD", 0, 50, 2);
	tft.drawRightString("A", 128, 50, 2);
	tft.drawString("Break", 0, 70, 2);
	tft.drawRightString("A", 128, 70, 2);
}

//Write Values, green when saved, red when new, current marked with white background
void drawSettingValues(uint16_t currentS){
	tft.setTextColor(RemoteData._deadband == EEPROM.get(eeDeadband, RemoteData._deadband) ? TFT_GREEN : TFT_RED, currentS == 1 ? TFT_WHITE : TFT_BLACK);
	tft.drawNumber(RemoteData._deadband, 80, 30, 2);
	
	tft.setTextColor(RemoteData._amp_fwd == EEPROM.get(eeFwd, RemoteData._amp_fwd) ? TFT_GREEN : TFT_RED, currentS == 2 ? TFT_WHITE : TFT_BLACK);
	tft.drawNumber(RemoteData._amp_fwd, 80, 50, 2);
	
	tft.setTextColor(RemoteData._amp_break == EEPROM.get(eeBreak, RemoteData._amp_break) ? TFT_GREEN : TFT_RED, currentS == 3 ? TFT_WHITE : TFT_BLACK);
	tft.drawNumber(RemoteData._amp_break, 80, 70, 2);
}

void SaveSettings(){
	EEPROM.update(eeDeadband, RemoteData._deadband);
	EEPROM.update(eeFwd, RemoteData._amp_fwd);
	EEPROM.update(eeBreak, RemoteData._amp_break);
}