/* ============================================================================
   ADJUSTABLE BENCH POWER SUPPLY - Combined firmware
   ============================================================================
   - Two rotary encoders (+push switches) set a Voltage target and a Current
     limit target. Pressing a switch moves which digit that encoder edits.
   - Voltage setpoint -> DAC channel A -> Vref -> analog op-amp loop -> pass
     transistors. This is the real, hardware voltage regulation path.
   - Current limit has no dedicated analog loop on this board, so it's
     enforced here in software: if measured current exceeds the limit, the
     DAC's voltage command is walked down until current backs off (a soft
     foldback CC, not a substitute for a hardware loop).
   - TFT shows Set V / Out V, Set I-limit / Out I, and CV/CC status.

   PINS - confirmed against the schematic AND the real board:
     TFT   CS   -> D9      TFT   D/C  -> D8
     DAC   CS   -> D10     DAC   DIN  -> MOSI (D11, hw SPI)
     DAC   SCLK -> SCK (D13, hw SPI)  TFT/DAC MISO -> D12 (hw SPI, unused)
     TFT   LED  -> tied to 3V3 on the board (no Arduino pin)
     TFT   RESET-> floating on the board (no Arduino pin)
     Voltage encoder: RE_1=D2, RE_2=D3, SW_1=D4
     Current encoder: RE_3=D6, RE_4=D7, SW_2=D5
     I_sense -> A0     V_sense -> A1
   ============================================================================ */

#include <SPI.h>
#include <LCDWIKI_GUI.h>
#include <LCDWIKI_SPI.h>

// ---------------------------------------------------------------------------
// PIN MAP (confirmed)
// ---------------------------------------------------------------------------
#define MODEL     ILI9341

#define TFT_CS     9
#define TFT_CD     8            // D/C
#define TFT_MISO  12            // hw SPI, unused by TFT but constructor wants it
#define TFT_MOSI  11            // hw SPI
#define TFT_RST   -1            // floating on this board
#define TFT_SCK   13            // hw SPI
#define TFT_LED   -1            // tied to 3V3 on this board

#define DAC_CS    10            // TLC5618A ~CS

#define ENC_V_A    2            // RE_1 (interrupt-capable)
#define ENC_V_B    3            // RE_2
#define ENC_V_SW   4            // SW_1

#define ENC_I_A    6            // RE_3 (interrupt-capable)
#define ENC_I_B    7            // RE_4
#define ENC_I_SW   5            // SW_2

#define ADC_ISENSE A0           // I_sense
#define ADC_VSENSE A1           // V_sense

// ---------------------------------------------------------------------------
// CALIBRATION CONSTANTS
// ---------------------------------------------------------------------------
const float DAC_VREF     = 1.090;          // V, DAC's REFIN voltage (= Arduino's internal ADC ref, shared)
const float V_OUT_GAIN   = 22.886;         // op-amp/feedback gain: DAC volts -> PSU output volts
const float ADC_VREF     = 1.090;          // Arduino internal ADC reference (analogReference(INTERNAL))
const float I_SENSE_GAIN = 0.216 * 3.425;  // shunt + diff-amp: volts at ADC pin per amp of output current

const float V_MAX = 30.0;
const float I_MAX = 3.0;

// ---------------------------------------------------------------------------
// TFT OBJECT (hardware SPI - pins passed match the real hw SPI pins, so the
// library uses the SPI peripheral rather than bit-banging)
// ---------------------------------------------------------------------------
LCDWIKI_SPI mylcd(MODEL, TFT_CS, TFT_CD, TFT_MISO, TFT_MOSI, TFT_RST, TFT_SCK, TFT_LED);

#define COL_BG    0x0000  // black
#define COL_LABEL 0x7BEF  // grey
#define COL_SETV  0x07FF  // cyan
#define COL_OUTV  0xFFFF  // white
#define COL_CV    0x07E0  // green
#define COL_CC    0xFFE0  // yellow
#define COL_CURSOR 0xFD20 // orange

// ---------------------------------------------------------------------------
// GLOBAL STATE
// ---------------------------------------------------------------------------
const float vSteps[] = {0.001, 0.01, 0.1, 1.0, 10.0};
const uint8_t V_DIGIT_COUNT = 5;
const float iSteps[] = {0.001, 0.01, 0.1, 1.0};
const uint8_t I_DIGIT_COUNT = 4;

int vDigit = 2;
int iDigit = 2;

float setVoltage      = 0.0;
float setCurrentLimit = 1.0;
float measuredVoltage = 0.0;
float measuredCurrent = 0.0;

bool  ccActive = false;
float ccBackoffVoltage = -1;

volatile int8_t encVDelta = 0;
volatile uint8_t encVLastState = 0;
volatile int8_t encIDelta = 0;
volatile uint8_t encILastState = 0;

unsigned long lastVSwPress = 0, lastISwPress = 0;
const unsigned long SW_DEBOUNCE_MS = 200;

unsigned long lastSenseMs = 0;
const unsigned long SENSE_INTERVAL_MS = 100;
unsigned long lastDisplayMs = 0;
const unsigned long DISPLAY_INTERVAL_MS = 150;

float lastDrawnSetV = -1, lastDrawnMeasV = -1;
float lastDrawnSetI = -1, lastDrawnMeasI = -1;
int   lastDrawnVDigit = -1, lastDrawnIDigit = -1;
bool  lastDrawnCC = false;

// ---------------------------------------------------------------------------
// DAC (TLC5618A) SPI DRIVER - hardware SPI, shares the bus with the TFT
// ---------------------------------------------------------------------------
// 16-bit word = [R1 SPD PWR R0][12-bit code]
//   R1,R0 = 10 -> write + update DAC A -> top nibble 1100 = 0xC (SPD=1,PWR=0)
//   R1,R0 = 00 -> write + update DAC B -> top nibble 0100 = 0x4 (SPD=1,PWR=0)
void dacWrite(uint8_t channel, uint16_t code) {
  if (code > 4095) code = 4095;
  uint16_t command = (channel == 0 ? 0xC000 : 0x4000) | (code & 0x0FFF);

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE2));
  digitalWrite(DAC_CS, LOW);
  SPI.transfer(command >> 8);
  SPI.transfer(command & 0xFF);
  digitalWrite(DAC_CS, HIGH);
  SPI.endTransaction();
}

uint16_t voltageToCode(float volts) {
  if (volts < 0) volts = 0;
  if (volts > V_MAX) volts = V_MAX;
  float dacVoltsNeeded = volts / V_OUT_GAIN;
  float code = (dacVoltsNeeded / (2.0 * DAC_VREF)) * 4096.0;
  if (code < 0) code = 0;
  if (code > 4095) code = 4095;
  return (uint16_t)(code + 0.5);
}

// ---------------------------------------------------------------------------
// ENCODER QUADRATURE DECODING (interrupt-driven)
// ---------------------------------------------------------------------------
void readEncoderV() {
  uint8_t state = (digitalRead(ENC_V_A) << 1) | digitalRead(ENC_V_B);
  if (state == encVLastState) return;
  if ((encVLastState == 0b00 && state == 0b01) ||
      (encVLastState == 0b01 && state == 0b11) ||
      (encVLastState == 0b11 && state == 0b10) ||
      (encVLastState == 0b10 && state == 0b00)) {
    encVDelta++;
  } else if ((encVLastState == 0b00 && state == 0b10) ||
             (encVLastState == 0b10 && state == 0b11) ||
             (encVLastState == 0b11 && state == 0b01) ||
             (encVLastState == 0b01 && state == 0b00)) {
    encVDelta--;
  }
  encVLastState = state;
}

void readEncoderI() {
  uint8_t state = (digitalRead(ENC_I_A) << 1) | digitalRead(ENC_I_B);
  if (state == encILastState) return;
  if ((encILastState == 0b00 && state == 0b01) ||
      (encILastState == 0b01 && state == 0b11) ||
      (encILastState == 0b11 && state == 0b10) ||
      (encILastState == 0b10 && state == 0b00)) {
    encIDelta++;
  } else if ((encILastState == 0b00 && state == 0b10) ||
             (encILastState == 0b10 && state == 0b11) ||
             (encILastState == 0b11 && state == 0b01) ||
             (encILastState == 0b01 && state == 0b00)) {
    encIDelta--;
  }
  encILastState = state;
}

// ---------------------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------------------
void setup() {
  pinMode(DAC_CS, OUTPUT);
  digitalWrite(DAC_CS, HIGH);
  SPI.begin();

  analogReference(INTERNAL);
  pinMode(ADC_VSENSE, INPUT);
  pinMode(ADC_ISENSE, INPUT);

  pinMode(ENC_V_A, INPUT_PULLUP);
  pinMode(ENC_V_B, INPUT_PULLUP);
  pinMode(ENC_V_SW, INPUT_PULLUP);
  pinMode(ENC_I_A, INPUT_PULLUP);
  pinMode(ENC_I_B, INPUT_PULLUP);
  pinMode(ENC_I_SW, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_V_A), readEncoderV, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_I_A), readEncoderI, CHANGE);

  mylcd.Init_LCD();
  mylcd.Set_Rotation(1);
  mylcd.Fill_Screen(COL_BG);
  drawStaticUI();

  dacWrite(0, voltageToCode(0));   // power up at 0V for safety
}

// ---------------------------------------------------------------------------
// MAIN LOOP (non-blocking)
// ---------------------------------------------------------------------------
void loop() {
  handleVoltageEncoder();
  handleCurrentEncoder();
  handleSwitches();

  if (millis() - lastSenseMs >= SENSE_INTERVAL_MS) {
    lastSenseMs = millis();
    readSensors();
    applySoftwareCurrentLimit();
  }

  if (millis() - lastDisplayMs >= DISPLAY_INTERVAL_MS) {
    lastDisplayMs = millis();
    updateDisplay();
  }
}

void handleVoltageEncoder() {
  noInterrupts();
  int8_t delta = encVDelta;
  encVDelta = 0;
  interrupts();
  if (delta == 0) return;

  setVoltage += delta * vSteps[vDigit];
  if (setVoltage < 0) setVoltage = 0;
  if (setVoltage > V_MAX) setVoltage = V_MAX;
  ccBackoffVoltage = -1;
  dacWrite(0, voltageToCode(setVoltage));
}

void handleCurrentEncoder() {
  noInterrupts();
  int8_t delta = encIDelta;
  encIDelta = 0;
  interrupts();
  if (delta == 0) return;

  setCurrentLimit += delta * iSteps[iDigit];
  if (setCurrentLimit < 0) setCurrentLimit = 0;
  if (setCurrentLimit > I_MAX) setCurrentLimit = I_MAX;
}

void handleSwitches() {
  if (digitalRead(ENC_V_SW) == LOW && millis() - lastVSwPress > SW_DEBOUNCE_MS) {
    lastVSwPress = millis();
    vDigit = (vDigit + 1) % V_DIGIT_COUNT;
  }
  if (digitalRead(ENC_I_SW) == LOW && millis() - lastISwPress > SW_DEBOUNCE_MS) {
    lastISwPress = millis();
    iDigit = (iDigit + 1) % I_DIGIT_COUNT;
  }
}

void readSensors() {
  int rawV = analogRead(ADC_VSENSE);
  int rawI = analogRead(ADC_ISENSE);
  measuredVoltage = (rawV * ADC_VREF * V_OUT_GAIN) / 1023.0;
  float senseVolts = (rawI * ADC_VREF) / 1023.0;
  measuredCurrent = senseVolts / I_SENSE_GAIN;
}

void applySoftwareCurrentLimit() {
  const float CC_STEP_V = 0.05;

  if (measuredCurrent > setCurrentLimit) {
    if (ccBackoffVoltage < 0) ccBackoffVoltage = setVoltage;
    ccBackoffVoltage -= CC_STEP_V;
    if (ccBackoffVoltage < 0) ccBackoffVoltage = 0;
    dacWrite(0, voltageToCode(ccBackoffVoltage));
    ccActive = true;
  } else if (ccActive) {
    ccActive = false;
    ccBackoffVoltage = -1;
    dacWrite(0, voltageToCode(setVoltage));
  }
}

// ---------------------------------------------------------------------------
// DISPLAY (LCDWIKI_GUI API: Fill_Rectangle/Draw_Rectangle take two corner
// points, not x/y/width/height)
// ---------------------------------------------------------------------------
// fixed pixel x-offsets of each digit within "TT.HHH" at text size 3 (18px/char)
// string layout: [0]T [1]T [2]. [3]H [4]H [5]H
const uint8_t vDigitCharPos[V_DIGIT_COUNT] = {5, 4, 3, 1, 0}; // idx0=thousandths..idx4=tens
// string layout for current "I.III": [0]I [1]. [2]I [3]I [4]I
const uint8_t iDigitCharPos[I_DIGIT_COUNT] = {4, 3, 2, 0};    // idx0=thousandths..idx3=ones

void drawStaticUI() {
  mylcd.Set_Text_Mode(0);
  mylcd.Set_Text_Back_colour(COL_BG);
  mylcd.Set_Text_colour(0xFFFF);
  mylcd.Set_Text_Size(2);
  mylcd.Print_String("BENCH POWER SUPPLY", 10, 8);

  mylcd.Set_Draw_color(COL_LABEL);
  mylcd.Draw_Line(0, 30, 320, 30);

  mylcd.Set_Text_colour(COL_LABEL);
  mylcd.Print_String("SET V", 10, 45);
  mylcd.Print_String("OUT V", 10, 100);
  mylcd.Print_String("SET I", 170, 45);
  mylcd.Print_String("OUT I", 170, 100);

  mylcd.Set_Draw_color(COL_LABEL);
  mylcd.Draw_Line(0, 150, 320, 150);
}

// Fixed-width zero-padded formatting (avoids sprintf's unreliable %f support
// on AVR/megaAVR cores - dtostrf is always available)
void formatFixed(char* buf, float value, uint8_t intDigits, uint8_t decDigits) {
  uint8_t width = intDigits + decDigits + 1;
  dtostrf(value, width, decDigits, buf);
  for (uint8_t i = 0; i < width; i++) {
    if (buf[i] == ' ') buf[i] = '0';
    else break;
  }
}

void drawField(int x, int y, float value, float &lastValue, uint16_t color,
               uint8_t intDigits, uint8_t decDigits, const char* unit) {
  if (fabs(value - lastValue) < 0.0005) return;
  lastValue = value;

  char buf[8];
  formatFixed(buf, value, intDigits, decDigits);

  mylcd.Set_Draw_color(COL_BG);
  mylcd.Fill_Rectangle(x, y, x + 150, y + 23);   // clear old text (two corner points)

  mylcd.Set_Text_Size(3);
  mylcd.Set_Text_colour(color);
  mylcd.Set_Text_Back_colour(COL_BG);
  mylcd.Print_String(buf, x, y);

  mylcd.Set_Text_Size(1);
  mylcd.Print_String(unit, x + (intDigits + decDigits + 1) * 18 + 4, y + 16);
}

void drawDigitCursor(int x, int y, int digit, int &lastDigit, const uint8_t* charPosTable) {
  if (digit == lastDigit) return;
  int charX = x + charPosTable[digit] * 18;

  mylcd.Set_Draw_color(COL_BG);
  mylcd.Fill_Rectangle(x, y + 26, x + 108, y + 28);       // clear old cursor strip

  mylcd.Set_Draw_color(COL_CURSOR);
  mylcd.Fill_Rectangle(charX, y + 26, charX + 17, y + 28); // draw new cursor mark

  lastDigit = digit;
}

void drawCCFlag() {
  if (ccActive == lastDrawnCC) return;
  lastDrawnCC = ccActive;

  mylcd.Set_Draw_color(COL_BG);
  mylcd.Fill_Rectangle(120, 200, 200, 230);

  mylcd.Set_Text_Size(2);
  mylcd.Set_Text_Back_colour(COL_BG);
  if (ccActive) {
    mylcd.Set_Text_colour(COL_CC);
    mylcd.Print_String("CC", 125, 208);
  } else {
    mylcd.Set_Text_colour(COL_CV);
    mylcd.Print_String("CV", 125, 208);
  }
}

void updateDisplay() {
  drawField(10, 65, setVoltage, lastDrawnSetV, COL_SETV, 2, 3, "V");
  drawDigitCursor(10, 65, vDigit, lastDrawnVDigit, vDigitCharPos);

  drawField(10, 120, measuredVoltage, lastDrawnMeasV, COL_OUTV, 2, 3, "V");

  drawField(170, 65, setCurrentLimit, lastDrawnSetI, COL_SETV, 1, 3, "A");
  drawDigitCursor(170, 65, iDigit, lastDrawnIDigit, iDigitCharPos);

  drawField(170, 120, measuredCurrent, lastDrawnMeasI, COL_OUTV, 1, 3, "A");

  drawCCFlag();
}
