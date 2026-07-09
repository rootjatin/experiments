#include <Adafruit_GFX.h>
#include <MCUFRIEND_kbv.h>
#include <TouchScreen.h>

MCUFRIEND_kbv tft;

// Common 2.4" MCUFRIEND ILI9341 touch pins
#define XP 6
#define XM A2
#define YP A1
#define YM 7

// Touch pressure
#define MINPRESSURE 200
#define MAXPRESSURE 1000

// Touch calibration
// Change these after calibration if touch position is wrong
#define TS_LEFT  907
#define TS_RT    136
#define TS_TOP   942
#define TS_BOT   139

TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);

// Colors
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

void setup()
{
  Serial.begin(9600);

  // Read TFT controller ID
  uint16_t ID = tft.readID();

  Serial.print("TFT ID = 0x");
  Serial.println(ID, HEX);

  // Start display
  tft.begin(ID);

  // Landscape mode
  tft.setRotation(1);

  // Clear screen
  tft.fillScreen(BLACK);

  // Title
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.setCursor(20, 10);
  tft.println("TOUCH TEST");

  tft.setTextColor(YELLOW);
  tft.setTextSize(1);
  tft.setCursor(20, 40);
  tft.println("Touch anywhere on screen");
}

void loop()
{
  TSPoint p = ts.getPoint();

  // Restore TFT pins because touch and TFT share pins
  pinMode(XM, OUTPUT);
  pinMode(YP, OUTPUT);

  if (p.z > MINPRESSURE && p.z < MAXPRESSURE)
  {
    // Convert raw touch value to screen coordinates
    int x = map(p.y, TS_LEFT, TS_RT, 0, tft.width());
    int y = map(p.x, TS_TOP, TS_BOT, 0, tft.height());

    // Keep coordinates inside display
    x = constrain(x, 0, tft.width() - 1);
    y = constrain(y, 0, tft.height() - 1);

    // Print touch data
    Serial.print("X = ");
    Serial.print(x);

    Serial.print(" Y = ");
    Serial.print(y);

    Serial.print(" Pressure = ");
    Serial.println(p.z);

    // Draw point where touched
    tft.fillCircle(x, y, 4, RED);

    delay(20);
  }
}
