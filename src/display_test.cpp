#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_SDA 8
#define OLED_SCL 9
#define OLED_ADDR 0x3C

Adafruit_SSD1306 display(128, 64, &Wire, -1);

void scanI2C() {
  Serial.println("I2C scan:");

  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("Found device: 0x");
      Serial.println(address, HEX);
    }
  }

  Serial.println("I2C scan complete");
}

void setup() {
  Serial.begin(115200);
  Wire.begin(OLED_SDA, OLED_SCL);
  Wire.setClock(400000);

  Serial.println();
  Serial.println("ESP32-S3 OLED display test");
  scanI2C();

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("ERROR: SSD1306 not found at 0x3C");
    Serial.println("Check wiring, power and try address 0x3D.");

    while (true) {
      delay(1000);
    }
  }

  Serial.println("SSD1306 initialized");

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("ESP32-S3");
  display.println("Display test");
  display.println();
  display.println("If you see this,");
  display.println("OLED works.");
  display.display();

  delay(2000);
}

void loop() {
  display.clearDisplay();

  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println("DISPLAY");
  display.println("TEST");

  display.drawRect(0, 40, 127, 23, SSD1306_WHITE);
  display.fillRect(2, 42, 20, 19, SSD1306_WHITE);
  display.drawCircle(80, 51, 10, SSD1306_WHITE);

  display.display();
  delay(1000);

  display.invertDisplay(true);
  delay(500);
  display.invertDisplay(false);
  delay(500);
}
