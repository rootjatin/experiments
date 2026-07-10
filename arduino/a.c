#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>

// TFT pins
#define TFT_CS   10
#define TFT_RST   9
#define TFT_DC    8

// Create TFT object
Adafruit_ST7735 tft = Adafruit_ST7735(
  TFT_CS,
  TFT_DC,
  TFT_RST
);

void setup()
{
  Serial.begin(9600);

  Serial.println("Starting TFT");

  // Initialize 1.8 inch ST7735 display
  tft.initR(INITR_BLACKTAB);

  // Screen rotation
  // 0, 1, 2 or 3
  tft.setRotation(1);

  // Clear screen
  tft.fillScreen(ST77XX_BLACK);

  // Set text color
  tft.setTextColor(ST77XX_WHITE);

  // Text size
  tft.setTextSize(2);

  // Text position
  tft.setCursor(10, 20);

  // Print text
  tft.println("Hello!");

  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(10, 50);
  tft.println("Arduino");

  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(10, 80);
  tft.println("1.8 TFT");
}

void loop()
{
}
