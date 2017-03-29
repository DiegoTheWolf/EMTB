
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

#define PIN_RADIO_CS 7
#define PIN_RADIO_CE 8
#define PIN_SD_CS 9
#define PIN

const uint8_t channel = 77;
const uint64_t pipe = 0x52582d5458; // 'RX-TX' pipe

struct point { // ToDo: Outsource
  uint8_t X;
  uint8_t Y;
};

struct point battery = {82, 54};
struct bldcMeasure VescMeasuredValues;

struct RemoteDataStruct {
  int8_t thr;
  bool cruise;
} RemoteData;

// Set up nRF24L01 radio on SPI bus
RF24 radio(PIN_RADIO_CS, PIN_RADIO_CE);

// Invoke library, pins defined in User_Setup.h
TFT_ST7735 tft = TFT_ST7735();

// functions
void DrawBattery(uint16_t color);
void FillBattery(uint8_t value);

void setup() {
  Serial.begin(115200);

  tft.init();
  tft.setRotation(0); // portrait
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  DrawBattery(TFT_WHITE);
  FillBattery(0);

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

  radio.startListening(); // Start listening
  radio.printDetails();   // Dump the configuration for debugging
  radio.powerUp();        // Leave low-power mode - making radio more responsive. // powerDown() for low-power
}

void loop() {
  // read Poti

  // read Button

  // send values to RX
  bool sendOK = radio.write(&RemoteData, sizeof(RemoteData));

  // recieve AckPayload
  while (radio.isAckPayloadAvailable()) {
    radio.read(&VescMeasuredValues, sizeof(VescMeasuredValues));
    bool recOK = true;
  }

  // Write Readings and AckPayload into a Logfile on the SD

  // Calculate Averages

  // Write Average-Values to screen (if changed)

  // Set LED Status (if changed)
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
