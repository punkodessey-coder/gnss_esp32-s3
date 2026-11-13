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

// RGB565 colors required by Arduino_GFX
#define BLACK  0x0000
#define WHITE  0xFFFF
#define RED    0xF800
#define GREEN  0x07E0
#define BLUE   0x001F

Arduino_DataBus *bus = new Arduino_ESP32SPI(LCD_DC, LCD_CS, LCD_SCLK, LCD_MOSI);
Arduino_GFX *gfx = new Arduino_ST7789(bus, LCD_RST, 1, true, LCD_WIDTH, LCD_HEIGHT, 34, 0, 34, 0);

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
  gfx->setCursor(70, 12);
  gfx->println("DISPLAY");
  gfx->println("TEST");

  gfx->fillRect(8, 70, 95, 25, RED);
  gfx->fillRect(112, 70, 95, 25, GREEN);
  gfx->fillRect(216, 70, 95, 25, BLUE);
  gfx->drawRect(0, 0, 319, 171, WHITE);

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
  gfx->setCursor(130, 80);
  gfx->println("TEST");

  delay(1000);
}
