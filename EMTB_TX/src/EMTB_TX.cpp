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

#include <Bounce2.h>
#include <EEPROM.h>
#include <RF24.h>
#include <RF24_config.h>
#include <SD.h>
#include <SPI.h>
#include <TFT_ST7735.h>
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
  Button Cruise <-- 2			A3 --> Poti(Throttle)
                        A4
Button Settings <-- 3			A2 --> Poti(FWD)
    CE nRF24L01 <-- 4			A1 --> Poti(Break)
    CS nRF24L01 <--	5			A0 --> Poti(TFT LED)
      CS SDcard <-- 6			13 --> nRF24L01 SCK  | SDcard SCK	 | TFT SCK
        CS TFT  <-- 7			12 --> nRF24L01 MISO | SDcard MISO
      Reset TFT <-- 8			11 --> nRF24L01 MOSI | SDcard MOSI | TFT SDA
         A0 TFT <--	9			10 --> TFT LED
*/

#define PIN_BTN_CRUISE 2
#define PIN_BTN_SETTINGS 3
#define PIN_RADIO_CS 4
#define PIN_RADIO_CE 5
#define PIN_SDCARD_CS 6
#define PIN_TFT_CS 7
#define PIN_TFT_RESET 8
#define PIN_TFT_A0 9
#define PIN_TFT_LED 10
#define PIN_POTI_THR A3
#define PIN_POTI_FWD A2
#define PIN_POTI_BREAK A1
#define PIN_POTI_LED A0

// constants
const uint8_t channel = 77;
const uint64_t pipe = 0x52582d5458;                                                                         // 'RX-TX' pipe
const uint32_t time_settings = 4095;                                                                        // [ms]
const uint8_t eeDeadband = 0;                                                                               // EEPROM Address
const uint8_t eeFwdMax = 1;                                                                                 // EEPROM Address
const uint8_t eeBreakMax = 2;                                                                               // EEPROM Address
const uint8_t eeFwdMin = 3;                                                                                 // EEPROM Address
const uint8_t eeBreakMin = 4;                                                                               // EEPROM Address
const uint8_t eeMaxVolt = 5;                                                                                // EEPROM Address
const uint16_t TFTrefresh = 500;                                                                            // [ms]
const uint16_t SDrefresh = 500;                                                                             // [ms]
const uint8_t wheelsize = 200;                                                                              // [mm]
const uint8_t gearratio = 3;                                                                                // [1:X]
const uint8_t pulse_rpm = 42;                                                                               // Number of poles * 3
const uint8_t erpm_rpm = 7;                                                                                 // Number of polse / 2
const float dist_corr_factor = 0.8;                                                                         // Number of polse / 2
const float ratio_RpmSpeed = (wheelsize * 3.141 * 60) / (erpm_rpm * gearratio * 1000000);                   // ERPM to km/h
const float ratio_TachoDist = ((wheelsize * 3.141) / (pulse_rpm * gearratio * 1000000)) * dist_corr_factor; // pulses to km
const uint16_t waitBeforeSend = 5000;                                                                       //[ms]

// globals
uint32_t TFTlastPaint;
uint32_t SDlastPrint;
uint32_t SettingsBtnPushTime;
uint32_t ridetime;
bool SendEnabled;
bool hasSDcard;
bool batteryNorm;
uint8_t amp_fwd_max;   // factor 0.5A
uint8_t amp_break_max; // factor 0.1A
uint8_t amp_fwd_min;   // factor 0.5A
uint8_t amp_break_min; // factor 0.1A
uint8_t maxvolt;       // factor 0.2A
uint8_t old_amp_fwd;
uint8_t old_amp_break;
uint8_t battery;
uint8_t old_battery;
uint8_t lastBatLine;

// average
uint16_t avgSum = 0;
const uint8_t avgCnt = 10;
uint8_t avg[avgCnt];
uint8_t avgIdx = 0;

File logfile;

struct RemoteDataStruct RemoteData;

struct bldcMeasure VescMeasuredValues;
struct bldcMeasure VescOldValues;

// functions
void drawLabels();
void drawValues();
uint16_t gradientRYG(uint8_t value);
void drawBattery(uint16_t color);
void fillBattery(uint8_t value);
void settingsMenu();
void changeSettings(bool up, uint16_t currentS);
void drawSettings();
void drawSettingValues(uint16_t currentS);
void saveSettings();
void discardSettings();

// objects
RF24 radio(PIN_RADIO_CS, PIN_RADIO_CE); // Set up nRF24L01 radio on SPI bus
TFT_ST7735 tft = TFT_ST7735();          // pins defined in User_Setup.h // ToDo: Move pin definition to this file
Bounce DEB_cruise = Bounce();

void setup() {

  pinMode(PIN_BTN_CRUISE, INPUT_PULLUP);
  pinMode(PIN_BTN_SETTINGS, INPUT_PULLUP);
  pinMode(PIN_POTI_THR, INPUT);
  pinMode(PIN_POTI_FWD, INPUT);
  pinMode(PIN_POTI_BREAK, INPUT);
  pinMode(PIN_POTI_LED, INPUT);
  pinMode(PIN_TFT_LED, OUTPUT);

  analogWrite(PIN_TFT_LED, analogRead(PIN_POTI_LED) >> 2);

  DEB_cruise.attach(PIN_BTN_CRUISE); // standard-interval 10 ms

  EEPROM.get(eeDeadband, RemoteData._deadband);
  EEPROM.get(eeFwdMax, amp_fwd_max);
  EEPROM.get(eeBreakMax, amp_break_max);
  EEPROM.get(eeFwdMin, amp_fwd_min);
  EEPROM.get(eeBreakMin, amp_break_min);
  EEPROM.get(eeMaxVolt, maxvolt);

  tft.init();
  tft.setRotation(0); // portrait
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.drawCentreString("Start", 64, 40, 2);
  tft.setTextSize(1);
  delay(2000);
  // Check if the button is pressed at startup.
  // Holding it down longer then "time_settings" will enter the settingsMenu and abort startup
  bool written = false;
  while (digitalRead(PIN_BTN_SETTINGS)) {
    if (!written) {
      tft.setTextColor(TFT_YELLOW, TFT_BLACK);
      tft.drawCentreString("Settings", 64, 100, 2);
      written = true;
    }
    if (millis() > time_settings) {
      settingsMenu();
    }
  }

  tft.fillRect(10, 100, 109, 16, TFT_BLACK); // Overwrite "Settings"
  tft.drawCentreString("Init", 64, 100, 2);

  if (!SD.begin(PIN_SDCARD_CS)) {
    tft.drawCentreString("No SD Card", 0, 130, 2);
  } else {
    hasSDcard = true;
    logfile = SD.open("logfile.txt", FILE_WRITE);
    if (!logfile)
      tft.drawCentreString("No SD Card", 0, 130, 2);
  }

  Serial.begin(115200);
  // while (!Serial()) {
  // } // Wait for Serial

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
  drawBattery(TFT_WHITE);
  drawLabels();
}

void loop() {
  // read POTI_THR and build average (we don't want a spiking throttle)
  avgSum -= avg[avgIdx];
  avg[avgIdx] = analogRead(PIN_POTI_THR);
  ;
  avgSum += avg[avgIdx];
  avgIdx++;
  if (avgIdx == avgCnt)
    avgIdx = 0;
  RemoteData.thr = avgSum / avgCnt;

  RemoteData._amp_fwd = map(analogRead(PIN_POTI_FWD), 0, 1023, amp_fwd_min, amp_fwd_max);
  RemoteData._amp_break = map(analogRead(PIN_POTI_BREAK), 0, 1023, amp_break_min, amp_break_max);

  // readButtons
  DEB_cruise.update();
  RemoteData.cruise = !DEB_cruise.read();

  if (SendEnabled) {
    // send values to RX
    radio.write(&RemoteData, sizeof(RemoteData));

    // recieve AckPayload
    while (radio.isAckPayloadAvailable()) {
      radio.read(&VescMeasuredValues, sizeof(VescMeasuredValues));
    }
  } else {
    if (millis() > waitBeforeSend)
      SendEnabled = true;
  }

  uint32_t _millis; // buffer 1x instead of 5x exec
                    // Write Readings and AckPayload into a Logfile on the SD
  bool SDsaved = false;
  if (hasSDcard) {
    _millis = millis();
    if (_millis > SDlastPrint + SDrefresh) {
      logfile.write((const uint8_t *)&RemoteData, sizeof(RemoteData));
      logfile.write((const uint8_t *)&VescMeasuredValues, sizeof(VescMeasuredValues));
      SDlastPrint = _millis;
      SDsaved = true;
    }
  }

  // Don't run SD save and TFT update in one loop. Both takes enough time.
  if (!SDsaved) {
    // Write Average-Values to screen (if changed)
    _millis = millis(); // buffer 1x instead of 5x exec
    if (_millis > TFTlastPaint + TFTrefresh) {
      if (digitalRead(PIN_BTN_SETTINGS)) {
        if (SettingsBtnPushTime == 0)
          SettingsBtnPushTime = _millis;
        if (_millis > SettingsBtnPushTime + time_settings) {
          SettingsBtnPushTime = 0;
          ridetime = _millis; // Reset Ridetime. This is inside the TFT Loop so it doesn't get called every loop.
        }
      } else {
        SettingsBtnPushTime = 0;
      }
      analogWrite(PIN_TFT_LED, analogRead(PIN_POTI_LED) >> 2); // Set TFT brightnes

      drawValues();
      VescOldValues = VescMeasuredValues;
      old_amp_fwd = RemoteData._amp_fwd;
      old_amp_break = RemoteData._amp_break;
      TFTlastPaint = _millis;
    }
  }
}

void drawLabels() {
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.drawCentreString("km", 73, 0, 2);
  tft.drawCentreString("h", 73, 15, 2);
  tft.drawFastHLine(65, 15, 16, TFT_CYAN);
  tft.drawCentreString("Motor", 22, 74, 2);
  tft.drawCentreString("Duty", 68, 74, 2);
  tft.drawString("km", 51, 95, 2);
  tft.drawCentreString(":", 38, 112, 2);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawRightString("F", 125, 128, 2);
  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.drawRightString("B", 125, 144, 2);
  tft.drawFastHLine(78, 126, 49, TFT_WHITE);
  tft.drawFastVLine(78, 126, 34, TFT_WHITE);
  tft.drawRect(4, 93, 66, 37, TFT_WHITE);
}

void drawValues() {
  tft.setTextSize(2);
  tft.setTextPadding(56); // xx (font4 * 2)
  tft.setTextColor(RemoteData.cruise ? TFT_GREEN : TFT_WHITE, TFT_BLACK);
  if (VescMeasuredValues.rpm != VescOldValues.rpm)
    tft.drawRightNumber(VescMeasuredValues.rpm * ratio_RpmSpeed, 62, 2, 4);

  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  old_battery = battery;
  battery = (VescMeasuredValues.v_in * 255) / (maxvolt / 5.0);
  if (old_battery != battery)
    fillBattery(battery);

  tft.setTextPadding(30); // xx,x (font2)
  if (RemoteData._amp_fwd != old_amp_fwd)
    tft.drawFloat(RemoteData._amp_fwd / 2.0, 1, 83, 128, 2);
  if (RemoteData._amp_break != old_amp_break)
    tft.drawFloat(RemoteData._amp_break / 10.0, 1, 83, 144, 2);

  tft.setTextPadding(38); // xx,xx (font2)
  if (VescMeasuredValues.tachometerAbs != VescOldValues.tachometerAbs)
    tft.drawFloat(VescMeasuredValues.tachometerAbs * ratio_TachoDist, 2, 10, 95, 2);

  tft.setTextPadding(42); // xxx (font4)
  if (VescMeasuredValues.v_in != VescOldValues.v_in)
    tft.drawFloat(VescMeasuredValues.v_in, 1, 77, 100, 4);
  if (VescMeasuredValues.current_motor != VescOldValues.current_motor)
    tft.drawCentreNumber(VescMeasuredValues.current_motor, 22, 51, 4);
  if (VescMeasuredValues.duty_now != VescOldValues.duty_now)
    tft.drawCentreNumber(VescMeasuredValues.duty_now, 68, 51, 4);
  float ampH = VescMeasuredValues.amp_hours - VescMeasuredValues.amp_hours_charged;
  float ampHold = VescOldValues.amp_hours - VescOldValues.amp_hours_charged;
  tft.setTextPadding(62); // xx,xx (font4)
  if (ampH != ampHold)
    tft.drawFloat(ampH, 2, 6, 134, 4);

  tft.setTextPadding(24); // xxx (font2)
  uint32_t _ridetime = (millis() - ridetime) / 1000;
  tft.drawRightNumber((_ridetime / 60), 35, 112, 2); // m
  tft.setTextPadding(16);                            // xx (font2)
  tft.drawNumber(_ridetime % 60, 42, 112, 2);        // s
}

// Return is an RGB value.
uint16_t gradientRYG(uint8_t value) {
  // From green to yellow G stays at 0xFF and R goes from 0x00 to 0xFF
  // Everything over yellow has R = 0xFF
  if (value < 128) {
    return 0xF800 + ((value * 2) << 3);
  } else {
    return 0x7E0 + (((255 - value) >> 2) << 11);
  }
}

void drawBattery(uint16_t color) {
  // fillRect(x, y, w, h, color);
  // Filling overlapping rectangles saves 86byte but is 9 times slower
  tft.drawRect(93, 0, 35, 97, color);
  tft.drawRect(94, 1, 33, 95, color);
}

void fillBattery(uint8_t value) {
  uint8_t line = 95 - (value / 2.8);
  if (line != lastBatLine) {
    lastBatLine = line;
    tft.fillRect(96, 3, 29, 91, TFT_BLACK); // Overwrite all previous
    if (value > 0) {
      if (!batteryNorm)
        drawBattery(TFT_WHITE);
      while (line <= 93) { // last line
        int16_t color_input;
        color_input = map(line, 3, 94, 255, 0);
        // drawFastHLine(x, y, w, color)
        tft.drawFastHLine(96, line, 29, gradientRYG(color_input));
        if (value < 25) {
          drawBattery(TFT_RED); // below 10% warning
          batteryNorm = false;
        }
        line++;
      }
    } else {
      drawBattery(TFT_YELLOW);
      batteryNorm = false;
    }
  }
}

// Enter Settings Mode. Exit only over reset.
void settingsMenu() {
  drawSettings();
  drawSettingValues(0);
  int8_t currentSetting = 0; // 0=save // 1=deadband // 2=amp_fwd_max // 3=amp_break_max // 4=amp_fwd_min // 5=amp_break_min // 6=maxvolt
  bool ok;
  bool triggerStick;
  uint16_t stick;
  while (1) {
    DEB_cruise.update();
    ok = !DEB_cruise.read();
    stick = analogRead(PIN_POTI_THR);
    // some movement has to be done to triggerStick
    // when button is pressed and stick moved change value.
    // without button change menu
    if (stick > 712 && !triggerStick) { // mid 512
      if (ok) {
        changeSettings(true, currentSetting);
        triggerStick = true;
      } else {
        currentSetting++;
        triggerStick = true;
      }
    } else if (stick < 312 && !triggerStick) {
      if (ok) {
        changeSettings(false, currentSetting);
        triggerStick = true;
      } else {
        currentSetting--;
        triggerStick = true;
      }
    } else if (stick < 612 && stick > 412 && triggerStick) { // 100 difference so it won't jitter
      // if (ok)
      //   changeSettings(false, currentSetting);
      triggerStick = false;
    }
    if (currentSetting > 6)
      currentSetting = 0;
    if (currentSetting < 0)
      currentSetting = 6;
    drawSettingValues(currentSetting);
  }
}

void changeSettings(bool up, uint16_t currentS) {
  switch (currentS) {
  case 0:
    up ? saveSettings() : discardSettings();
    // up ? saveSettings() : discardSettings();
    break;
  case 1:
    up ? RemoteData._deadband++ : RemoteData._deadband--;
    break;
  case 2:
    up ? amp_fwd_max++ : amp_fwd_max--;
    break;
  case 3:
    up ? amp_break_max++ : amp_break_max--;
    break;
  case 4:
    up ? amp_fwd_min++ : amp_fwd_min--;
    break;
  case 5:
    up ? amp_break_min++ : amp_break_min--;
    break;
  case 6:
    up ? maxvolt++ : maxvolt--;
    break;
  }
}

void drawSettings() {
  tft.setTextSize(1); // no scaling
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.drawCentreString("SETTINGS", 64, 5, 4); // Font 4

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("Deadband", 5, 35, 2);
  tft.drawString("FWD max", 5, 55, 2);
  tft.drawRightString("A", 120, 55, 2);
  tft.drawString("Break max", 5, 75, 2);
  tft.drawRightString("A", 120, 75, 2);
  tft.drawString("FWD min", 5, 95, 2);
  tft.drawRightString("A", 120, 95, 2);
  tft.drawString("Break min", 5, 115, 2);
  tft.drawRightString("A", 120, 115, 2);
  tft.drawString("Max Volt", 5, 135, 2);
}

// Write Values, green when saved, red when new, current marked with white background
void drawSettingValues(uint16_t currentS) {
  uint8_t eeBuffer;
  tft.setTextPadding(24); // for 255
  EEPROM.get(eeDeadband, eeBuffer);
  tft.setTextColor(RemoteData._deadband == eeBuffer ? TFT_GREEN : TFT_RED, currentS == 1 ? TFT_LIGHTGREY : TFT_BLACK);
  tft.drawNumber(RemoteData._deadband, 85, 35, 2);

  tft.setTextPadding(38); // for 127,5
  EEPROM.get(eeFwdMax, eeBuffer);
  tft.setTextColor(amp_fwd_max == eeBuffer ? TFT_GREEN : TFT_RED, currentS == 2 ? TFT_LIGHTGREY : TFT_BLACK);
  tft.drawFloat(amp_fwd_max / 2.0, 1, 85, 55, 2);

  EEPROM.get(eeFwdMin, eeBuffer);
  tft.setTextColor(amp_fwd_min == eeBuffer ? TFT_GREEN : TFT_RED, currentS == 4 ? TFT_LIGHTGREY : TFT_BLACK);
  tft.drawFloat(amp_fwd_min / 2.0, 1, 85, 95, 2);

  tft.setTextPadding(30); // for 25,5
  EEPROM.get(eeBreakMax, eeBuffer);
  tft.setTextColor(amp_break_max == eeBuffer ? TFT_GREEN : TFT_RED, currentS == 3 ? TFT_LIGHTGREY : TFT_BLACK);
  tft.drawFloat(amp_break_max / 10.0, 1, 85, 75, 2);

  EEPROM.get(eeBreakMin, eeBuffer);
  tft.setTextColor(amp_break_min == eeBuffer ? TFT_GREEN : TFT_RED, currentS == 5 ? TFT_LIGHTGREY : TFT_BLACK);
  tft.drawFloat(amp_break_min / 10.0, 1, 85, 115, 2);

  EEPROM.get(eeMaxVolt, eeBuffer);
  tft.setTextColor(maxvolt == eeBuffer ? TFT_GREEN : TFT_RED, currentS == 6 ? TFT_LIGHTGREY : TFT_BLACK);
  tft.drawNumber(maxvolt / 5.0, 85, 135, 2);
}

void saveSettings() {
  EEPROM.update(eeDeadband, RemoteData._deadband);
  EEPROM.update(eeFwdMax, amp_fwd_max);
  EEPROM.update(eeBreakMax, amp_break_max);
  EEPROM.update(eeFwdMin, amp_fwd_min);
  EEPROM.update(eeBreakMin, amp_break_min);
  EEPROM.update(eeMaxVolt, maxvolt);
}

void discardSettings() {
  EEPROM.get(eeDeadband, RemoteData._deadband);
  EEPROM.get(eeFwdMax, amp_fwd_max);
  EEPROM.get(eeBreakMax, amp_break_max);
  EEPROM.get(eeFwdMin, amp_fwd_min);
  EEPROM.get(eeBreakMin, amp_break_min);
  EEPROM.get(eeMaxVolt, maxvolt);
}
