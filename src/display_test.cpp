#include <Arduino.h>
#include <Arduino_GFX_Library.h>

#define LCD_MOSI 45
#define LCD_SCLK 40
#define LCD_CS 42
#define LCD_DC 41
#define LCD_RST 39
#define LCD_BL 46
#define LCD_WIDTH 172
#define LCD_HEIGHT 320

Arduino_DataBus *bus = new Arduino_ESP32SPI(LCD_DC, LCD_CS, LCD_SCLK, LCD_MOSI);
Arduino_GFX *gfx = new Arduino_ST7789(bus, LCD_RST, 0, true, LCD_WIDTH, LCD_HEIGHT, 34, 0, 34, 0);

void setup() {
  Serial.begin(115200);

  pinMode(LCD_BL, OUTPUT);
  digitalWrite(LCD_BL, HIGH);

  if (!gfx->begin()) {
    Serial.println("ST7789 initialization failed");
    while (true) delay(1000);
  }

  gfx->fillScreen(BLACK);
  gfx->setTextColor(WHITE);
  gfx->setTextSize(2);
  gfx->setCursor(10, 20);
  gfx->println("DISPLAY");
  gfx->println("TEST");

  gfx->fillRect(10, 90, 152, 40, RED);
  gfx->fillRect(10, 140, 152, 40, GREEN);
  gfx->fillRect(10, 190, 152, 40, BLUE);
  gfx->drawRect(0, 0, 171, 319, WHITE);

  Serial.println("Display test running");
}

void loop() {
  static uint8_t color = 0;
  color = (color + 1) % 3;

  if (color == 0) gfx->fillScreen(RED);
  else if (color == 1) gfx->fillScreen(GREEN);
  else gfx->fillScreen(BLUE);

  gfx->setTextColor(WHITE);
  gfx->setTextSize(3);
  gfx->setCursor(30, 140);
  gfx->println("TEST");

  delay(1000);
}
