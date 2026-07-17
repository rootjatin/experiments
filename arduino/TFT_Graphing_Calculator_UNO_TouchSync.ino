#include <Adafruit_GFX.h>
#include <MCUFRIEND_kbv.h>
#include <TouchScreen.h>
#include <EEPROM.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

// This project is specifically for Arduino UNO R3 / ATmega328P.
// It deliberately stops compilation when an ESP32 or another MCU is selected.
#if defined(ARDUINO) && !defined(__AVR_ATmega328P__)
  #error "Select Tools > Board > Arduino AVR Boards > Arduino Uno"
#endif

/*
  TFT Graphing Calculator
  -----------------------
  Target: Arduino UNO R3 (ATmega328P) with MCUFRIEND parallel TFT shield
          and 4-wire resistive touch panel.

  Libraries:
    - MCUFRIEND_kbv
    - Adafruit GFX Library
    - Adafruit TouchScreen

  Main features:
    - 9-point touchscreen calibration stored in EEPROM
    - Piecewise affine touch mapping to reduce edge offset and local distortion
    - Median touch filtering and relaxed pressure limits
    - On-screen expression keypad
    - Parser for +, -, *, /, ^, parentheses, x, pi, e
    - Functions: sin, cos, tan, sqrt, log, exp, abs
    - Graph plotting, horizontal pan, and zoom

  First boot:
    The calibration wizard starts automatically.

  Recalibration:
    Reset the Arduino and touch/hold the screen while the startup message is
    visible. Calibration is stored in EEPROM after all 9 points are collected.
*/

MCUFRIEND_kbv tft;

// Common UNO-style MCUFRIEND resistive-touch pin assignment.
const int XP = 6;
const int XM = A2;
const int YP = A1;
const int YM = 7;

TouchScreen ts(XP, YP, XM, YM, 300);

// Display orientation. This sketch is designed for portrait mode.
const uint8_t SCREEN_ROTATION = 0;

// Touch settings. MAX_PRESSURE is intentionally generous because many
// low-cost resistive panels report a much larger Z value near the bezel.
const int16_t MIN_PRESSURE = 60;
const int16_t MAX_PRESSURE = 4000;
const uint8_t TOUCH_SAMPLE_COUNT = 3;
const uint8_t TOUCH_REQUIRED_SAMPLES = 2;
const int16_t BUTTON_HIT_EXPAND = 5;

// Colors in RGB565.
const uint16_t COLOR_BLACK   = 0x0000;
const uint16_t COLOR_WHITE   = 0xFFFF;
const uint16_t COLOR_GRAY    = 0x8410;
const uint16_t COLOR_DARKGRAY= 0x4208;
const uint16_t COLOR_BLUE    = 0x001F;
const uint16_t COLOR_CYAN    = 0x07FF;
const uint16_t COLOR_GREEN   = 0x07E0;
const uint16_t COLOR_RED     = 0xF800;
const uint16_t COLOR_YELLOW  = 0xFFE0;
const uint16_t COLOR_ORANGE  = 0xFD20;

// Calibration geometry. The targets use equal margins on every side.
// Older versions used an asymmetric top margin and then snapped edge touches,
// which caused the touch position to drift and become unusable near borders.
const uint8_t CAL_GRID = 3;
const uint8_t CAL_POINT_COUNT = 9;
const int16_t CAL_X_MARGIN = 16;
const int16_t CAL_Y_MARGIN = 16;
const uint32_t CAL_MAGIC = 0x47524333UL;  // "GRC3" forces fresh calibration

struct CalibrationData {
  uint32_t magic;
  int16_t rawX[CAL_POINT_COUNT];
  int16_t rawY[CAL_POINT_COUNT];
  uint16_t checksum;
};

CalibrationData calibration;

// Custom data types must appear before the first function in an .ino file.
// Arduino automatically generates function prototypes, so placing these types
// here prevents generated prototypes from referring to unknown types.
struct FloatPoint {
  float x;
  float y;
};

struct Button {
  int16_t x;
  int16_t y;
  int16_t w;
  int16_t h;
};

// Expression buffer. Keep this modest for Arduino UNO SRAM.
const uint8_t MAX_EXPR_LEN = 47;
char expression[MAX_EXPR_LEN + 1] = "sin(x)";
uint8_t expressionLength = 6;

// Graph view.
float viewXMin = -10.0f;
float viewXMax =  10.0f;
float viewYMin = -10.0f;
float viewYMax =  10.0f;

const int16_t GRAPH_HEADER_H = 20;
const int16_t GRAPH_TOOLBAR_H = 40;

enum ScreenMode {
  MODE_EDIT_MAIN,
  MODE_EDIT_FUNCTIONS,
  MODE_GRAPH
};

ScreenMode screenMode = MODE_EDIT_MAIN;
bool touchWasDown = false;
unsigned long lastTouchReleaseMs = 0;

// -----------------------------------------------------------------------------
// Utility helpers
// -----------------------------------------------------------------------------

int16_t median5(int16_t *values, uint8_t count) {
  for (uint8_t i = 1; i < count; ++i) {
    int16_t key = values[i];
    int8_t j = (int8_t)i - 1;
    while (j >= 0 && values[j] > key) {
      values[j + 1] = values[j];
      --j;
    }
    values[j + 1] = key;
  }
  return values[count / 2];
}

uint16_t calibrationChecksum(const CalibrationData &data) {
  uint32_t sum = 0x5A5A;
  for (uint8_t i = 0; i < CAL_POINT_COUNT; ++i) {
    sum += (uint16_t)data.rawX[i];
    sum = (sum << 1) | (sum >> 31);
    sum += (uint16_t)data.rawY[i];
  }
  return (uint16_t)(sum ^ (sum >> 16));
}

bool calibrationIsValid() {
  if (calibration.magic != CAL_MAGIC) return false;
  if (calibration.checksum != calibrationChecksum(calibration)) return false;

  // Reject erased EEPROM and implausibly tiny calibration spans.
  int16_t minX = calibration.rawX[0];
  int16_t maxX = calibration.rawX[0];
  int16_t minY = calibration.rawY[0];
  int16_t maxY = calibration.rawY[0];

  for (uint8_t i = 1; i < CAL_POINT_COUNT; ++i) {
    if (calibration.rawX[i] < minX) minX = calibration.rawX[i];
    if (calibration.rawX[i] > maxX) maxX = calibration.rawX[i];
    if (calibration.rawY[i] < minY) minY = calibration.rawY[i];
    if (calibration.rawY[i] > maxY) maxY = calibration.rawY[i];
  }

  return (maxX - minX > 250) && (maxY - minY > 250);
}

void restoreSharedPins() {
  pinMode(YP, OUTPUT);
  pinMode(XM, OUTPUT);
}

bool readOneRawTouch(int16_t &rawX, int16_t &rawY, int16_t &pressure) {
  TSPoint point = ts.getPoint();
  restoreSharedPins();

  pressure = point.z;
  if (pressure < MIN_PRESSURE || pressure > MAX_PRESSURE) return false;

  rawX = point.x;
  rawY = point.y;
  return true;
}

bool readFilteredRawTouch(int16_t &rawX, int16_t &rawY, int16_t &pressure) {
  int16_t xs[TOUCH_SAMPLE_COUNT];
  int16_t ys[TOUCH_SAMPLE_COUNT];
  int16_t zs[TOUCH_SAMPLE_COUNT];
  uint8_t count = 0;

  for (uint8_t attempt = 0;
       attempt < TOUCH_SAMPLE_COUNT + 4 && count < TOUCH_SAMPLE_COUNT;
       ++attempt) {
    int16_t x, y, z;
    if (readOneRawTouch(x, y, z)) {
      xs[count] = x;
      ys[count] = y;
      zs[count] = z;
      ++count;
    }
    delayMicroseconds(180);
  }

  if (count < TOUCH_REQUIRED_SAMPLES) return false;

  rawX = median5(xs, count);
  rawY = median5(ys, count);
  pressure = median5(zs, count);
  return true;
}

int16_t calibrationScreenX(uint8_t column) {
  if (column == 0) return CAL_X_MARGIN;
  if (column == 1) return (tft.width() - 1) / 2;
  return tft.width() - 1 - CAL_X_MARGIN;
}

int16_t calibrationScreenY(uint8_t row) {
  if (row == 0) return CAL_Y_MARGIN;
  if (row == 1) return (tft.height() - 1) / 2;
  return tft.height() - 1 - CAL_Y_MARGIN;
}

void drawCalibrationCross(int16_t x, int16_t y, uint16_t color) {
  tft.drawFastHLine(x - 8, y, 17, color);
  tft.drawFastVLine(x, y - 8, 17, color);
  tft.drawCircle(x, y, 4, color);
}

void waitForRawRelease() {
  unsigned long releasedSince = 0;
  while (true) {
    int16_t x, y, z;
    bool down = readOneRawTouch(x, y, z);
    if (!down) {
      if (releasedSince == 0) releasedSince = millis();
      if (millis() - releasedSince > 120) return;
    } else {
      releasedSince = 0;
    }
    delay(8);
  }
}

bool collectCalibrationPoint(int16_t &outX, int16_t &outY) {
  // Median calibration rejects a single bad sample much better than averaging.
  const uint8_t needed = 15;
  const uint8_t ignoredAtStart = 3;
  int16_t xs[needed];
  int16_t ys[needed];
  uint8_t stableSamples = 0;
  uint8_t ignoredSamples = 0;
  unsigned long start = millis();

  while (millis() - start < 15000UL) {
    int16_t x, y, z;
    if (readOneRawTouch(x, y, z)) {
      if (ignoredSamples < ignoredAtStart) {
        ++ignoredSamples;
      } else if (stableSamples < needed) {
        xs[stableSamples] = x;
        ys[stableSamples] = y;
        ++stableSamples;
      }

      if (stableSamples >= needed) {
        outX = median5(xs, needed);
        outY = median5(ys, needed);
        waitForRawRelease();
        return true;
      }
    } else {
      stableSamples = 0;
      ignoredSamples = 0;
    }
    delay(5);
  }

  return false;
}

void runCalibrationWizard() {
  // Show instructions first, then remove all text while collecting points.
  // Text near a target can make it difficult to touch the exact cross center.
  tft.fillScreen(COLOR_BLACK);
  tft.setTextColor(COLOR_WHITE, COLOR_BLACK);
  tft.setTextSize(1);
  tft.setCursor(20, 135);
  tft.print("Touch and hold each cross");
  tft.setCursor(28, 153);
  tft.print("Use a narrow plastic stylus");
  delay(1200);

  for (uint8_t row = 0; row < CAL_GRID; ++row) {
    for (uint8_t column = 0; column < CAL_GRID; ++column) {
      uint8_t index = row * CAL_GRID + column;
      int16_t sx = calibrationScreenX(column);
      int16_t sy = calibrationScreenY(row);

      tft.fillScreen(COLOR_BLACK);
      drawCalibrationCross(sx, sy, COLOR_YELLOW);

      int16_t rx = 0;
      int16_t ry = 0;
      while (!collectCalibrationPoint(rx, ry)) {
        // A short red flash indicates that the contact was not stable enough.
        drawCalibrationCross(sx, sy, COLOR_RED);
        delay(180);
        drawCalibrationCross(sx, sy, COLOR_YELLOW);
      }

      calibration.rawX[index] = rx;
      calibration.rawY[index] = ry;
      drawCalibrationCross(sx, sy, COLOR_GREEN);
      delay(180);
      waitForRawRelease();
    }
  }

  calibration.magic = CAL_MAGIC;
  calibration.checksum = calibrationChecksum(calibration);
  EEPROM.put(0, calibration);

  tft.fillScreen(COLOR_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(COLOR_GREEN, COLOR_BLACK);
  tft.setCursor(24, 130);
  tft.print("Calibration saved");
  delay(900);
}

// -----------------------------------------------------------------------------
// Piecewise affine touch transformation
// -----------------------------------------------------------------------------

float outsidePenalty(float a, float b, float c) {
  float penalty = 0.0f;
  if (a < 0.0f) penalty -= a;
  if (b < 0.0f) penalty -= b;
  if (c < 0.0f) penalty -= c;
  if (a > 1.0f) penalty += a - 1.0f;
  if (b > 1.0f) penalty += b - 1.0f;
  if (c > 1.0f) penalty += c - 1.0f;
  return penalty;
}

bool barycentric(
  const FloatPoint &p,
  const FloatPoint &a,
  const FloatPoint &b,
  const FloatPoint &c,
  float &wa,
  float &wb,
  float &wc
) {
  float denominator = (b.y - c.y) * (a.x - c.x) +
                      (c.x - b.x) * (a.y - c.y);
  if (fabs(denominator) < 0.0001f) return false;

  wa = ((b.y - c.y) * (p.x - c.x) +
        (c.x - b.x) * (p.y - c.y)) / denominator;

  wb = ((c.y - a.y) * (p.x - c.x) +
        (a.x - c.x) * (p.y - c.y)) / denominator;

  wc = 1.0f - wa - wb;
  return true;
}

void calibrationRawPoint(uint8_t index, FloatPoint &point) {
  point.x = calibration.rawX[index];
  point.y = calibration.rawY[index];
}

void calibrationDisplayPoint(uint8_t index, FloatPoint &point) {
  uint8_t row = index / CAL_GRID;
  uint8_t column = index % CAL_GRID;
  point.x = calibrationScreenX(column);
  point.y = calibrationScreenY(row);
}

void testTriangleMapping(
  const FloatPoint &rawPoint,
  uint8_t ia,
  uint8_t ib,
  uint8_t ic,
  float &bestPenalty,
  float &bestX,
  float &bestY
) {
  FloatPoint ra, rb, rc;
  FloatPoint sa, sb, sc;
  calibrationRawPoint(ia, ra);
  calibrationRawPoint(ib, rb);
  calibrationRawPoint(ic, rc);
  calibrationDisplayPoint(ia, sa);
  calibrationDisplayPoint(ib, sb);
  calibrationDisplayPoint(ic, sc);

  float wa, wb, wc;
  if (!barycentric(rawPoint, ra, rb, rc, wa, wb, wc)) return;

  float penalty = outsidePenalty(wa, wb, wc);
  if (penalty < bestPenalty) {
    bestPenalty = penalty;
    bestX = wa * sa.x + wb * sb.x + wc * sc.x;
    bestY = wa * sa.y + wb * sb.y + wc * sc.y;
  }
}

bool mapRawToScreen(int16_t rawX, int16_t rawY, int16_t &screenX, int16_t &screenY) {
  FloatPoint rawPoint;
  rawPoint.x = rawX;
  rawPoint.y = rawY;

  float bestPenalty = 1000000.0f;
  float bestX = 0.0f;
  float bestY = 0.0f;

  // Four grid cells, two triangles per cell.
  for (uint8_t row = 0; row < 2; ++row) {
    for (uint8_t column = 0; column < 2; ++column) {
      uint8_t p00 = row * 3 + column;
      uint8_t p10 = p00 + 1;
      uint8_t p01 = p00 + 3;
      uint8_t p11 = p01 + 1;

      testTriangleMapping(rawPoint, p00, p10, p01,
                          bestPenalty, bestX, bestY);
      testTriangleMapping(rawPoint, p10, p11, p01,
                          bestPenalty, bestX, bestY);
    }
  }

  // A touch outside the inner calibration rectangle needs controlled
  // extrapolation to reach the physical bezel. Keep a generous rejection
  // threshold so valid edge touches are not treated as dead zones.
  if (bestPenalty > 12.0f) return false;

  const float xStart = (float)CAL_X_MARGIN;
  const float xEnd = (float)(tft.width() - 1 - CAL_X_MARGIN);
  const float yStart = (float)CAL_Y_MARGIN;
  const float yEnd = (float)(tft.height() - 1 - CAL_Y_MARGIN);

  // Convert the mapped inner calibration rectangle to the full display.
  // Unlike snapping, this preserves motion and scale all the way to each edge.
  float fullX = (bestX - xStart) * (float)(tft.width() - 1) /
                (xEnd - xStart);
  float fullY = (bestY - yStart) * (float)(tft.height() - 1) /
                (yEnd - yStart);

  screenX = constrain((int16_t)(fullX + 0.5f),
                      (int16_t)0,
                      (int16_t)(tft.width() - 1));
  screenY = constrain((int16_t)(fullY + 0.5f),
                      (int16_t)0,
                      (int16_t)(tft.height() - 1));

  return true;
}

bool getTouch(int16_t &screenX, int16_t &screenY, int16_t &pressure) {
  int16_t rawX, rawY;
  if (!readFilteredRawTouch(rawX, rawY, pressure)) return false;
  return mapRawToScreen(rawX, rawY, screenX, screenY);
}

// -----------------------------------------------------------------------------
// Expression parser
// -----------------------------------------------------------------------------

class ExpressionParser {
public:
  ExpressionParser(const char *text, float xValue)
    : cursor(text), x(xValue), valid(true) {}

  float evaluate() {
    float value = parseExpression();
    skipSpaces();
    if (*cursor != '\0') valid = false;
    if (isnan(value) || isinf(value)) valid = false;
    return value;
  }

  bool isValid() const { return valid; }

private:
  const char *cursor;
  float x;
  bool valid;

  void skipSpaces() {
    while (*cursor == ' ' || *cursor == '\t') ++cursor;
  }

  bool consume(char expected) {
    skipSpaces();
    if (*cursor == expected) {
      ++cursor;
      return true;
    }
    return false;
  }

  float parseExpression() {
    float value = parseTerm();
    while (valid) {
      if (consume('+')) value += parseTerm();
      else if (consume('-')) value -= parseTerm();
      else break;
    }
    return value;
  }

  float parseTerm() {
    float value = parseUnary();
    while (valid) {
      if (consume('*')) {
        value *= parseUnary();
      } else if (consume('/')) {
        float divisor = parseUnary();
        if (fabs(divisor) < 0.0000001f) {
          valid = false;
          return 0.0f;
        }
        value /= divisor;
      } else {
        break;
      }
    }
    return value;
  }

  float parseUnary() {
    skipSpaces();
    if (consume('+')) return parseUnary();
    if (consume('-')) return -parseUnary();
    return parsePower();
  }

  float parsePower() {
    float base = parsePrimary();
    if (consume('^')) {
      float exponent = parseUnary();
      base = pow(base, exponent);
      if (isnan(base) || isinf(base)) valid = false;
    }
    return base;
  }

  float parsePrimary() {
    skipSpaces();

    if (consume('(')) {
      float value = parseExpression();
      if (!consume(')')) valid = false;
      return value;
    }

    if (isdigit(*cursor) || *cursor == '.') {
      char *endPointer;
      double value = strtod(cursor, &endPointer);
      if (endPointer == cursor) {
        valid = false;
        return 0.0f;
      }
      cursor = endPointer;
      return (float)value;
    }

    if (isalpha(*cursor)) {
      char identifier[8];
      uint8_t length = 0;
      while (isalpha(*cursor) && length < sizeof(identifier) - 1) {
        identifier[length++] = (char)tolower(*cursor);
        ++cursor;
      }
      identifier[length] = '\0';

      if (strcmp(identifier, "x") == 0) return x;
      if (strcmp(identifier, "pi") == 0) return PI;
      if (strcmp(identifier, "e") == 0) return 2.718281828f;

      if (!consume('(')) {
        valid = false;
        return 0.0f;
      }

      float argument = parseExpression();
      if (!consume(')')) {
        valid = false;
        return 0.0f;
      }

      if (strcmp(identifier, "sin") == 0) return sin(argument);
      if (strcmp(identifier, "cos") == 0) return cos(argument);
      if (strcmp(identifier, "tan") == 0) return tan(argument);
      if (strcmp(identifier, "sqrt") == 0) {
        if (argument < 0.0f) valid = false;
        return sqrt(argument);
      }
      if (strcmp(identifier, "log") == 0) {
        if (argument <= 0.0f) valid = false;
        return log(argument);
      }
      if (strcmp(identifier, "exp") == 0) return exp(argument);
      if (strcmp(identifier, "abs") == 0) return fabs(argument);

      valid = false;
      return 0.0f;
    }

    valid = false;
    return 0.0f;
  }
};

bool evaluateExpression(const char *text, float x, float &result) {
  ExpressionParser parser(text, x);
  result = parser.evaluate();
  return parser.isValid();
}

// -----------------------------------------------------------------------------
// UI helpers
// -----------------------------------------------------------------------------

bool pointInButton(int16_t px, int16_t py, const Button &button) {
  return px >= button.x - BUTTON_HIT_EXPAND &&
         px <  button.x + button.w + BUTTON_HIT_EXPAND &&
         py >= button.y - BUTTON_HIT_EXPAND &&
         py <  button.y + button.h + BUTTON_HIT_EXPAND;
}

void drawButton(const Button &button, const char *label, uint16_t fillColor) {
  tft.fillRect(button.x + 1, button.y + 1,
               button.w - 2, button.h - 2, fillColor);
  tft.drawRect(button.x, button.y, button.w, button.h, COLOR_WHITE);

  uint8_t textSize = 1;
  size_t labelLength = strlen(label);
  if (labelLength <= 3 && button.w >= 45 && button.h >= 35) textSize = 2;

  int16_t textWidth = (int16_t)(labelLength * 6 * textSize);
  int16_t textHeight = 8 * textSize;
  int16_t tx = button.x + (button.w - textWidth) / 2;
  int16_t ty = button.y + (button.h - textHeight) / 2;

  tft.setTextSize(textSize);
  tft.setTextColor(COLOR_WHITE, fillColor);
  tft.setCursor(tx, ty);
  tft.print(label);
}

Button keypadButton(uint8_t row, uint8_t column) {
  const int16_t top = 62;
  const int16_t columns = 4;
  const int16_t rows = 6;
  int16_t width = tft.width() / columns;
  int16_t height = (tft.height() - top) / rows;

  Button button;
  button.x = column * width;
  button.y = top + row * height;
  button.w = (column == columns - 1) ? tft.width() - button.x : width;
  button.h = (row == rows - 1) ? tft.height() - button.y : height;
  return button;
}

void drawExpressionBox() {
  tft.fillRect(0, 0, tft.width(), 60, COLOR_BLACK);
  tft.setTextColor(COLOR_CYAN, COLOR_BLACK);
  tft.setTextSize(1);
  tft.setCursor(4, 3);
  tft.print(screenMode == MODE_EDIT_MAIN ? "EDIT" : "FUNCTIONS");
  tft.print("  y=");

  tft.drawRect(3, 16, tft.width() - 6, 39, COLOR_WHITE);
  tft.setTextColor(COLOR_YELLOW, COLOR_BLACK);
  tft.setTextSize(1);
  tft.setCursor(7, 23);

  // Show the tail if the expression is too long for the field.
  const uint8_t visibleCharacters = 36;
  const char *start = expression;
  if (expressionLength > visibleCharacters) {
    start = expression + expressionLength - visibleCharacters;
  }
  tft.print(start);
}

const char *mainLabels[24] = {
  "7", "8", "9", "/",
  "4", "5", "6", "*",
  "1", "2", "3", "-",
  "0", ".", "x", "+",
  "(", ")", "^", "DEL",
  "FUNC", "CLR", "PLOT", "pi"
};

const char *functionLabels[24] = {
  "sin(", "cos(", "tan(", "sqrt(",
  "log(", "exp(", "abs(", "pi",
  "x", "^", "(", ")",
  "+", "-", "*", "/",
  "DEL", "CLR", "MAIN", "PLOT",
  "x", "x^2", "sin(x)", "1/x"
};

void drawEditScreen() {
  tft.fillScreen(COLOR_BLACK);
  drawExpressionBox();

  const char **labels =
    (screenMode == MODE_EDIT_MAIN) ? mainLabels : functionLabels;

  for (uint8_t row = 0; row < 6; ++row) {
    for (uint8_t column = 0; column < 4; ++column) {
      uint8_t index = row * 4 + column;
      Button button = keypadButton(row, column);
      uint16_t fill = COLOR_DARKGRAY;

      if (strcmp(labels[index], "PLOT") == 0) fill = COLOR_GREEN;
      else if (strcmp(labels[index], "CLR") == 0) fill = COLOR_RED;
      else if (strcmp(labels[index], "FUNC") == 0 ||
               strcmp(labels[index], "MAIN") == 0) fill = COLOR_BLUE;

      drawButton(button, labels[index], fill);
    }
  }
}

bool appendText(const char *text) {
  uint8_t addedLength = strlen(text);
  if (expressionLength + addedLength > MAX_EXPR_LEN) return false;
  memcpy(expression + expressionLength, text, addedLength + 1);
  expressionLength += addedLength;
  return true;
}

void setExpression(const char *text) {
  strncpy(expression, text, MAX_EXPR_LEN);
  expression[MAX_EXPR_LEN] = '\0';
  expressionLength = strlen(expression);
}

void deleteLastCharacter() {
  if (expressionLength == 0) return;
  --expressionLength;
  expression[expressionLength] = '\0';
}

void showEditMessage(const char *message, uint16_t color) {
  tft.fillRect(4, 44, tft.width() - 8, 10, COLOR_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(color, COLOR_BLACK);
  tft.setCursor(7, 45);
  tft.print(message);
}

float niceGridStep(float range) {
  if (range <= 0.0f) return 1.0f;
  float rough = range / 6.0f;
  float exponent = floor(log10(rough));
  float power = pow(10.0f, exponent);
  float fraction = rough / power;

  if (fraction < 1.5f) return 1.0f * power;
  if (fraction < 3.5f) return 2.0f * power;
  if (fraction < 7.5f) return 5.0f * power;
  return 10.0f * power;
}

int16_t graphLeft() { return 0; }
int16_t graphRight() { return tft.width() - 1; }
int16_t graphTop() { return GRAPH_HEADER_H; }
int16_t graphBottom() { return tft.height() - GRAPH_TOOLBAR_H - 1; }
int16_t graphWidth() { return graphRight() - graphLeft() + 1; }
int16_t graphHeight() { return graphBottom() - graphTop() + 1; }

int16_t worldToPixelX(float x) {
  float fraction = (x - viewXMin) / (viewXMax - viewXMin);
  return graphLeft() + (int16_t)(fraction * (graphWidth() - 1));
}

int16_t worldToPixelY(float y) {
  float fraction = (viewYMax - y) / (viewYMax - viewYMin);
  return graphTop() + (int16_t)(fraction * (graphHeight() - 1));
}

float pixelToWorldX(int16_t px) {
  float fraction = (float)(px - graphLeft()) / (graphWidth() - 1);
  return viewXMin + fraction * (viewXMax - viewXMin);
}

void drawGraphGrid() {
  tft.fillRect(graphLeft(), graphTop(), graphWidth(), graphHeight(), COLOR_BLACK);

  float xStep = niceGridStep(viewXMax - viewXMin);
  float yStep = niceGridStep(viewYMax - viewYMin);

  float firstX = ceil(viewXMin / xStep) * xStep;
  for (float x = firstX; x <= viewXMax + 0.5f * xStep; x += xStep) {
    int16_t px = worldToPixelX(x);
    if (px >= graphLeft() && px <= graphRight()) {
      tft.drawFastVLine(px, graphTop(), graphHeight(), COLOR_DARKGRAY);
    }
  }

  float firstY = ceil(viewYMin / yStep) * yStep;
  for (float y = firstY; y <= viewYMax + 0.5f * yStep; y += yStep) {
    int16_t py = worldToPixelY(y);
    if (py >= graphTop() && py <= graphBottom()) {
      tft.drawFastHLine(graphLeft(), py, graphWidth(), COLOR_DARKGRAY);
    }
  }

  if (viewXMin <= 0.0f && viewXMax >= 0.0f) {
    int16_t axisX = worldToPixelX(0.0f);
    tft.drawFastVLine(axisX, graphTop(), graphHeight(), COLOR_WHITE);
  }

  if (viewYMin <= 0.0f && viewYMax >= 0.0f) {
    int16_t axisY = worldToPixelY(0.0f);
    tft.drawFastHLine(graphLeft(), axisY, graphWidth(), COLOR_WHITE);
  }
}

bool validateExpression() {
  float result;
  return evaluateExpression(expression, 0.37f, result);
}

void plotExpression() {
  bool havePrevious = false;
  int16_t previousX = 0;
  int16_t previousY = 0;

  for (int16_t px = graphLeft(); px <= graphRight(); ++px) {
    float x = pixelToWorldX(px);
    float y;
    bool valid = evaluateExpression(expression, x, y);

    if (!valid || isnan(y) || isinf(y)) {
      havePrevious = false;
      continue;
    }

    float normalized = (viewYMax - y) / (viewYMax - viewYMin);
    float pyFloat = graphTop() + normalized * (graphHeight() - 1);

    // Keep a guard band so asymptotes do not create giant vertical lines.
    if (pyFloat < graphTop() - graphHeight() ||
        pyFloat > graphBottom() + graphHeight()) {
      havePrevious = false;
      continue;
    }

    int16_t py = (int16_t)pyFloat;

    if (havePrevious) {
      int16_t jump = abs(py - previousY);
      if (jump < graphHeight() / 2) {
        tft.drawLine(previousX, previousY, px, py, COLOR_YELLOW);
      }
    }

    previousX = px;
    previousY = py;
    havePrevious = true;
  }
}

Button toolbarButton(uint8_t index) {
  const uint8_t count = 6;
  int16_t y = tft.height() - GRAPH_TOOLBAR_H;
  int16_t width = tft.width() / count;

  Button button;
  button.x = index * width;
  button.y = y;
  button.w = (index == count - 1) ? tft.width() - button.x : width;
  button.h = GRAPH_TOOLBAR_H;
  return button;
}

const char *toolbarLabels[6] = {"EDIT", "+", "-", "<", ">", "HOME"};

void drawGraphScreen() {
  tft.fillScreen(COLOR_BLACK);

  tft.setTextSize(1);
  tft.setTextColor(COLOR_CYAN, COLOR_BLACK);
  tft.setCursor(3, 4);
  tft.print("y=");
  const uint8_t maxHeaderChars = 34;
  const char *start = expression;
  if (expressionLength > maxHeaderChars) {
    start = expression + expressionLength - maxHeaderChars;
  }
  tft.print(start);

  drawGraphGrid();
  plotExpression();

  for (uint8_t i = 0; i < 6; ++i) {
    uint16_t fill = COLOR_DARKGRAY;
    if (i == 0) fill = COLOR_BLUE;
    if (i == 5) fill = COLOR_GREEN;
    drawButton(toolbarButton(i), toolbarLabels[i], fill);
  }
}

void zoomGraph(float factor) {
  float centerX = (viewXMin + viewXMax) * 0.5f;
  float centerY = (viewYMin + viewYMax) * 0.5f;
  float halfX = (viewXMax - viewXMin) * 0.5f * factor;
  float halfY = (viewYMax - viewYMin) * 0.5f * factor;

  halfX = constrain(halfX, 0.05f, 100000.0f);
  halfY = constrain(halfY, 0.05f, 100000.0f);

  viewXMin = centerX - halfX;
  viewXMax = centerX + halfX;
  viewYMin = centerY - halfY;
  viewYMax = centerY + halfY;
}

void panGraph(float fraction) {
  float shift = (viewXMax - viewXMin) * fraction;
  viewXMin += shift;
  viewXMax += shift;
}

void resetGraphView() {
  viewXMin = -10.0f;
  viewXMax =  10.0f;
  viewYMin = -10.0f;
  viewYMax =  10.0f;
}

void handleMainKey(const char *label) {
  if (strcmp(label, "FUNC") == 0) {
    screenMode = MODE_EDIT_FUNCTIONS;
    drawEditScreen();
    return;
  }
  if (strcmp(label, "CLR") == 0) {
    setExpression("");
    drawExpressionBox();
    return;
  }
  if (strcmp(label, "DEL") == 0) {
    deleteLastCharacter();
    drawExpressionBox();
    return;
  }
  if (strcmp(label, "PLOT") == 0) {
    if (expressionLength == 0 || !validateExpression()) {
      showEditMessage("Invalid expression", COLOR_RED);
      return;
    }
    screenMode = MODE_GRAPH;
    drawGraphScreen();
    return;
  }

  if (!appendText(label)) {
    showEditMessage("Expression is full", COLOR_RED);
  } else {
    drawExpressionBox();
  }
}

void handleFunctionKey(uint8_t index, const char *label) {
  if (strcmp(label, "MAIN") == 0) {
    screenMode = MODE_EDIT_MAIN;
    drawEditScreen();
    return;
  }
  if (strcmp(label, "CLR") == 0) {
    setExpression("");
    drawExpressionBox();
    return;
  }
  if (strcmp(label, "DEL") == 0) {
    deleteLastCharacter();
    drawExpressionBox();
    return;
  }
  if (strcmp(label, "PLOT") == 0) {
    if (expressionLength == 0 || !validateExpression()) {
      showEditMessage("Invalid expression", COLOR_RED);
      return;
    }
    screenMode = MODE_GRAPH;
    drawGraphScreen();
    return;
  }

  // Last row contains complete presets.
  if (index == 20) setExpression("x");
  else if (index == 21) setExpression("x^2");
  else if (index == 22) setExpression("sin(x)");
  else if (index == 23) setExpression("1/x");
  else if (!appendText(label)) {
    showEditMessage("Expression is full", COLOR_RED);
    return;
  }

  drawExpressionBox();
}

void handleEditTouch(int16_t x, int16_t y) {
  if (y < 62) return;

  for (uint8_t row = 0; row < 6; ++row) {
    for (uint8_t column = 0; column < 4; ++column) {
      uint8_t index = row * 4 + column;
      Button button = keypadButton(row, column);
      if (pointInButton(x, y, button)) {
        if (screenMode == MODE_EDIT_MAIN) {
          handleMainKey(mainLabels[index]);
        } else {
          handleFunctionKey(index, functionLabels[index]);
        }
        return;
      }
    }
  }
}

void handleGraphTouch(int16_t x, int16_t y) {
  if (y < tft.height() - GRAPH_TOOLBAR_H) return;

  for (uint8_t i = 0; i < 6; ++i) {
    Button button = toolbarButton(i);
    if (!pointInButton(x, y, button)) continue;

    switch (i) {
      case 0:
        screenMode = MODE_EDIT_MAIN;
        drawEditScreen();
        break;
      case 1:
        zoomGraph(0.5f);
        drawGraphScreen();
        break;
      case 2:
        zoomGraph(2.0f);
        drawGraphScreen();
        break;
      case 3:
        panGraph(-0.25f);
        drawGraphScreen();
        break;
      case 4:
        panGraph(0.25f);
        drawGraphScreen();
        break;
      case 5:
        resetGraphView();
        drawGraphScreen();
        break;
    }
    return;
  }
}

bool startupTouchRequestsCalibration() {
  tft.fillScreen(COLOR_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(COLOR_WHITE, COLOR_BLACK);
  tft.setCursor(8, 120);
  tft.print("Hold screen to recalibrate");
  tft.setCursor(8, 138);
  tft.print("Starting in 1.5 seconds...");

  unsigned long start = millis();
  unsigned long heldSince = 0;

  while (millis() - start < 1500UL) {
    int16_t rx, ry, z;
    bool down = readOneRawTouch(rx, ry, z);
    if (down) {
      if (heldSince == 0) heldSince = millis();
      if (millis() - heldSince > 600UL) {
        waitForRawRelease();
        return true;
      }
    } else {
      heldSince = 0;
    }
    delay(10);
  }

  return false;
}

void setup() {
  Serial.begin(115200);

  uint16_t displayID = tft.readID();
  if (displayID == 0xD3D3) displayID = 0x9486;

  tft.begin(displayID);
  tft.setRotation(SCREEN_ROTATION);
  tft.fillScreen(COLOR_BLACK);

  EEPROM.get(0, calibration);
  bool forceCalibration = startupTouchRequestsCalibration();

  if (!calibrationIsValid() || forceCalibration) {
    runCalibrationWizard();
  }

  drawEditScreen();

  Serial.print("TFT ID: 0x");
  Serial.println(displayID, HEX);
  Serial.println("Graphing calculator ready.");
}

void loop() {
  int16_t x, y, pressure;
  bool touchDown = getTouch(x, y, pressure);

  // Trigger exactly once on a new press. A release delay helps prevent
  // resistive-panel bounce from creating two button presses.
  if (touchDown && !touchWasDown && millis() - lastTouchReleaseMs > 70UL) {
    if (screenMode == MODE_GRAPH) handleGraphTouch(x, y);
    else handleEditTouch(x, y);

    Serial.print("Touch X=");
    Serial.print(x);
    Serial.print(" Y=");
    Serial.print(y);
    Serial.print(" Z=");
    Serial.println(pressure);
  }

  if (!touchDown && touchWasDown) {
    lastTouchReleaseMs = millis();
  }

  touchWasDown = touchDown;
  delay(8);
}
