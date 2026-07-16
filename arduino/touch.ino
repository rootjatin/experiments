#include <Adafruit_GFX.h>
#include <MCUFRIEND_kbv.h>
#include <TouchScreen.h>

MCUFRIEND_kbv tft;

// ==========================================================
// Touchscreen pins
// ==========================================================

const int XP = 6;
const int XM = A2;
const int YP = A1;
const int YM = 7;

TouchScreen ts(XP, YP, XM, YM, 300);

// ==========================================================
// Calibration
// ==========================================================

/*
  X-axis three-point calibration.

  Open Serial Monitor and touch:
  1. Physical left edge
  2. Exact horizontal center
  3. Physical right edge

  Replace these values with the average Raw X readings.

  These starting values are based on your existing calibration.
*/
const int16_t RAW_X_LEFT   = 136;
const int16_t RAW_X_CENTER = 522;
const int16_t RAW_X_RIGHT  = 907;

/*
  Y-axis calibration.

  Your current Y direction works with these values.
*/
const int16_t RAW_Y_TOP    = 942;
const int16_t RAW_Y_BOTTOM = 139;

// Small final pixel adjustments only.
const int16_t X_OFFSET_PIXELS = 0;
const int16_t Y_OFFSET_PIXELS = 9;

/*
  Edge snapping should remain small.

  Use 0 while calibrating.
  After calibration, values from 2 to 4 are suitable.
*/
const int16_t EDGE_SNAP_PIXELS = 3;

// ==========================================================
// General settings
// ==========================================================

const int MIN_PRESSURE = 150;
const int MAX_PRESSURE = 1000;

const uint8_t SCREEN_ROTATION = 0;

const uint16_t BACKGROUND_COLOR = 0x0000;
const uint16_t PEN_COLOR = 0xFFFF;

bool penIsDown = false;

int16_t previousX = 0;
int16_t previousY = 0;

unsigned long lastSerialPrint = 0;

// ==========================================================
// Median filter
// ==========================================================

int16_t medianOfThree(int16_t a, int16_t b, int16_t c)
{
  if (a > b) {
    int16_t temporary = a;
    a = b;
    b = temporary;
  }

  if (b > c) {
    int16_t temporary = b;
    b = c;
    c = temporary;
  }

  if (a > b) {
    int16_t temporary = a;
    a = b;
    b = temporary;
  }

  return b;
}

// ==========================================================
// Linear axis mapping
// ==========================================================

int16_t mapTouchAxis(
  int16_t rawValue,
  int16_t rawAtStart,
  int16_t rawAtEnd,
  int16_t pixelMaximum)
{
  if (rawAtStart == rawAtEnd) {
    return 0;
  }

  int32_t numerator =
    (int32_t)(rawValue - rawAtStart) * pixelMaximum;

  int32_t denominator =
    (int32_t)rawAtEnd - rawAtStart;

  return (int16_t)(numerator / denominator);
}

// ==========================================================
// Three-point axis mapping
// ==========================================================

int16_t mapThreePointAxis(
  int16_t rawValue,
  int16_t rawAtStart,
  int16_t rawAtCenter,
  int16_t rawAtEnd,
  int16_t pixelMaximum)
{
  int16_t pixelCenter = pixelMaximum / 2;

  bool valueIsInFirstHalf;

  /*
    Support both increasing and decreasing raw coordinates.
  */
  if (rawAtStart < rawAtEnd) {
    valueIsInFirstHalf = rawValue <= rawAtCenter;
  }
  else {
    valueIsInFirstHalf = rawValue >= rawAtCenter;
  }

  if (valueIsInFirstHalf) {
    return mapTouchAxis(
      rawValue,
      rawAtStart,
      rawAtCenter,
      pixelCenter
    );
  }

  int16_t secondHalfPixel = mapTouchAxis(
    rawValue,
    rawAtCenter,
    rawAtEnd,
    pixelMaximum - pixelCenter
  );

  return pixelCenter + secondHalfPixel;
}

// ==========================================================
// Raw touchscreen reading
// ==========================================================

bool getRawTouch(
  int16_t &rawX,
  int16_t &rawY,
  int16_t &pressure)
{
  TSPoint point = ts.getPoint();

  /*
    Restore shared display pins after reading touchscreen.
  */
  pinMode(YP, OUTPUT);
  pinMode(XM, OUTPUT);

  pressure = point.z;

  if (pressure < MIN_PRESSURE ||
      pressure > MAX_PRESSURE) {
    return false;
  }

  rawX = point.x;
  rawY = point.y;

  return true;
}

// ==========================================================
// Final coordinate correction
// ==========================================================

void correctCoordinates(int16_t &x, int16_t &y)
{
  int16_t maximumX = tft.width() - 1;
  int16_t maximumY = tft.height() - 1;

  x += X_OFFSET_PIXELS;
  y += Y_OFFSET_PIXELS;

  x = constrain(x, 0, maximumX);
  y = constrain(y, 0, maximumY);

  if (EDGE_SNAP_PIXELS > 0) {

    if (x <= EDGE_SNAP_PIXELS) {
      x = 0;
    }
    else if (x >= maximumX - EDGE_SNAP_PIXELS) {
      x = maximumX;
    }

    if (y <= EDGE_SNAP_PIXELS) {
      y = 0;
    }
    else if (y >= maximumY - EDGE_SNAP_PIXELS) {
      y = maximumY;
    }
  }
}

// ==========================================================
// Filtered touchscreen reading
// ==========================================================

bool getFilteredTouch(
  int16_t &screenX,
  int16_t &screenY,
  int16_t &pressure,
  int16_t &rawX,
  int16_t &rawY)
{
  int16_t rawXValues[3];
  int16_t rawYValues[3];
  int16_t pressureValues[3];

  uint8_t validReadings = 0;

  /*
    Obtain three valid samples.
    Extra attempts are allowed if one reading is noisy.
  */
  for (
    uint8_t attempt = 0;
    attempt < 7 && validReadings < 3;
    attempt++
  ) {
    int16_t sampleX;
    int16_t sampleY;
    int16_t samplePressure;

    if (getRawTouch(
          sampleX,
          sampleY,
          samplePressure)) {

      rawXValues[validReadings] = sampleX;
      rawYValues[validReadings] = sampleY;
      pressureValues[validReadings] = samplePressure;

      validReadings++;
    }

    delayMicroseconds(200);
  }

  if (validReadings < 3) {
    return false;
  }

  rawX = medianOfThree(
    rawXValues[0],
    rawXValues[1],
    rawXValues[2]
  );

  rawY = medianOfThree(
    rawYValues[0],
    rawYValues[1],
    rawYValues[2]
  );

  pressure = medianOfThree(
    pressureValues[0],
    pressureValues[1],
    pressureValues[2]
  );

  /*
    X uses three-point calibration because the error
    was increasing toward the right side.
  */
  screenX = mapThreePointAxis(
    rawX,
    RAW_X_LEFT,
    RAW_X_CENTER,
    RAW_X_RIGHT,
    tft.width() - 1
  );

  /*
    Y uses normal two-point calibration.
  */
  screenY = mapTouchAxis(
    rawY,
    RAW_Y_TOP,
    RAW_Y_BOTTOM,
    tft.height() - 1
  );

  correctCoordinates(screenX, screenY);

  return true;
}

// ==========================================================
// Setup
// ==========================================================

void setup()
{
  Serial.begin(115200);

  uint16_t displayID = tft.readID();

  if (displayID == 0xD3D3) {
    displayID = 0x9486;
  }

  Serial.print("TFT ID: 0x");
  Serial.println(displayID, HEX);

  tft.begin(displayID);
  tft.setRotation(SCREEN_ROTATION);
  tft.fillScreen(BACKGROUND_COLOR);

  Serial.println();
  Serial.println("Three-point touchscreen drawing test");
  Serial.println();
  Serial.println("Touch left, center and right.");
  Serial.println("Record the Raw X values.");
  Serial.println("Reset Arduino to clear the display.");
  Serial.println();
}

// ==========================================================
// Main drawing loop
// ==========================================================

void loop()
{
  int16_t x;
  int16_t y;

  int16_t rawX;
  int16_t rawY;
  int16_t pressure;

  bool touched = getFilteredTouch(
    x,
    y,
    pressure,
    rawX,
    rawY
  );

  if (touched) {

    if (!penIsDown) {
      /*
        First point of a new stroke.
        drawPixel draws exactly one display pixel.
      */
      tft.drawPixel(x, y, PEN_COLOR);
    }
    else {
      int16_t movementX = abs(x - previousX);
      int16_t movementY = abs(y - previousY);

      /*
        Connect normal movement with a one-pixel-wide line.
        Ignore very large jumps caused by noise.
      */
      if (movementX <= 50 &&
          movementY <= 50) {

        tft.drawLine(
          previousX,
          previousY,
          x,
          y,
          PEN_COLOR
        );
      }
      else {
        tft.drawPixel(x, y, PEN_COLOR);
      }
    }

    previousX = x;
    previousY = y;

    penIsDown = true;

    /*
      Print readings without slowing drawing too much.
    */
    if (millis() - lastSerialPrint >= 40) {
      lastSerialPrint = millis();

      Serial.print("Raw X=");
      Serial.print(rawX);

      Serial.print(" Raw Y=");
      Serial.print(rawY);

      Serial.print(" Pressure=");
      Serial.print(pressure);

      Serial.print(" Screen X=");
      Serial.print(x);

      Serial.print(" Screen Y=");
      Serial.println(y);
    }
  }
  else {
    /*
      The next touch begins a new stroke.
    */
    penIsDown = false;
  }
}
