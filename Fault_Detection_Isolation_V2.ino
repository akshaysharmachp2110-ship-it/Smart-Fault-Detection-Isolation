/*
  Smart Microcontroller-Based Fault Detection and Automatic Isolation
  System for Electrical Load Protection   (Version 2 - with voltage sensing)
  Board : Arduino Uno

  PIN CONNECTIONS
  ------------------------------------------------------------
  ACS712 (current) OUT  -> A0
  ZMPT101B (voltage) OUT-> A1
  LCD I2C               -> SDA = A4, SCL = A5
  Relay IN              -> D7
  Buzzer (+)            -> D8
  Red LED 1             -> D11  (via 220 ohm)
  Red LED 2             -> D10  (via 220 ohm)
  Green LED 1           -> D12  (via 220 ohm)
  Green LED 2           -> D13  (via 220 ohm)

  LIBRARY NEEDED
  ------------------------------------------------------------
  "LiquidCrystal I2C" by Frank de Brabander

  PROTECTION LOGIC
  ------------------------------------------------------------
  Overload      : I >= 0.42 A for 10 s            -> trip
  Short circuit : I >= 2.00 A (instant)           -> trip
  Over voltage  : V >= 250 V for 3 s              -> trip
  Under voltage : V <= 170 V for 3 s              -> trip
  On trip: relay OFF (load isolated), both RED LEDs ON, buzzer ON,
  both GREEN LEDs OFF, LCD shows the fault.

  IMPORTANT: connect the ZMPT101B on the SUPPLY side (before the relay)
  so voltage can still be measured after the load is isolated.
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ---------------- Pin definitions ----------------
#define CURRENT_PIN   A0
#define VOLTAGE_PIN   A1
#define RELAY_PIN     7
#define BUZZER_PIN    8
#define RED_LED_1     2
#define RED_LED_2     3
#define GREEN_LED_1   12
#define GREEN_LED_2   13

// ---------------- Relay logic ----------------
// Most 5V relay modules are ACTIVE-LOW (LOW = relay ON).
// If your relay works the opposite way, swap these two values.
#define RELAY_ON      LOW
#define RELAY_OFF     HIGH

// ---------------- Sensor settings ----------------
// ACS712-05B : 0.185 | ACS712-20A : 0.100 | ACS712-30A : 0.066  (V per Amp)
const float SENSITIVITY    = 0.185;
const float ADC_REF        = 5.0;
const unsigned long SAMPLE_WINDOW = 100;   // ms (5 cycles of 50 Hz)

// ZMPT101B calibration factor:
//   VOLT_CALIB = (actual mains voltage measured with a multimeter)
//                / (voltage shown on LCD / Serial)  x  current VOLT_CALIB
// Start with 500.0, then adjust until the LCD matches your multimeter.
float VOLT_CALIB           = 500.0;

const float CURRENT_NOISE_FLOOR = 0.10;    // A, below this shown as 0
const float VOLTAGE_NOISE_FLOOR = 20.0;    // V, below this shown as 0

// ---------------- Protection settings ----------------
const float OVERLOAD_LIMIT       = 0.42;   // A
const float SHORT_CIRCUIT_LIMIT  = 2.00;   // A (instant trip)
const unsigned long OVERLOAD_TIME_MS = 10000;  // overload must last 10 s
const unsigned long INRUSH_DELAY_MS  = 1000;   // ignore current after switch ON

const bool  ENABLE_VOLTAGE_PROTECTION = true;  // set false until calibrated
const float OVER_VOLTAGE_LIMIT   = 250.0;  // V
const float UNDER_VOLTAGE_LIMIT  = 170.0;  // V
const unsigned long VOLT_FAULT_TIME_MS = 3000; // voltage fault must last 3 s

const bool  AUTO_RETRY           = false;  // true = auto restore after delay
const unsigned long RETRY_DELAY_MS = 10000;

// ---------------- Variables ----------------
LiquidCrystal_I2C lcd(0x27, 16, 2);        // try 0x3F if LCD stays blank

enum Fault { NO_FAULT, OVERLOAD_FAULT, SHORT_FAULT, OVERVOLT_FAULT, UNDERVOLT_FAULT };
Fault systemState = NO_FAULT;

float currentRMS = 0.0;
float voltageRMS = 0.0;

unsigned long relayOnTime   = 0;
unsigned long faultTime     = 0;
unsigned long overloadStart = 0;
unsigned long voltStart     = 0;
bool overloadTiming         = false;
bool voltTiming             = false;
Fault voltTimingType        = NO_FAULT;
unsigned long lastLcdUpdate = 0;

// ---------------- Prototypes ----------------
void readSensors();
void loadON();
void tripSystem(Fault f);
void setGreen(bool on);
void setRed(bool on);
void updateDisplay();
bool voltageInRange();

// =====================================================
void setup() {
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RED_LED_1, OUTPUT);
  pinMode(RED_LED_2, OUTPUT);
  pinMode(GREEN_LED_1, OUTPUT);
  pinMode(GREEN_LED_2, OUTPUT);

  digitalWrite(RELAY_PIN, RELAY_OFF);      // load isolated at start
  digitalWrite(BUZZER_PIN, LOW);
  setGreen(false);
  setRed(false);

  Serial.begin(9600);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Fault Detection");
  lcd.setCursor(0, 1);
  lcd.print("& Isolation Sys");
  delay(2000);

  lcd.clear();
  lcd.print("Checking supply");
  readSensors();                           // warm-up readings
  readSensors();
  delay(500);

  tone(BUZZER_PIN, 2000, 150);             // start-up beep
  delay(300);
  loadON();
}

// =====================================================
void loop() {
  readSensors();

  if (systemState == NO_FAULT) {

    // ---------- Current protection ----------
    if (millis() - relayOnTime > INRUSH_DELAY_MS) {

      if (currentRMS >= SHORT_CIRCUIT_LIMIT) {
        tripSystem(SHORT_FAULT);
      }
      else if (currentRMS >= OVERLOAD_LIMIT) {
        if (!overloadTiming) {
          overloadTiming = true;
          overloadStart = millis();
        } else if (millis() - overloadStart >= OVERLOAD_TIME_MS) {
          tripSystem(OVERLOAD_FAULT);
        }
      }
      else {
        overloadTiming = false;
      }
    }

    // ---------- Voltage protection ----------
    if (systemState == NO_FAULT && ENABLE_VOLTAGE_PROTECTION) {
      Fault vf = NO_FAULT;
      if (voltageRMS >= OVER_VOLTAGE_LIMIT)       vf = OVERVOLT_FAULT;
      else if (voltageRMS <= UNDER_VOLTAGE_LIMIT) vf = UNDERVOLT_FAULT;

      if (vf == NO_FAULT) {
        voltTiming = false;
      } else {
        if (!voltTiming || voltTimingType != vf) {
          voltTiming = true;
          voltTimingType = vf;
          voltStart = millis();
        } else if (millis() - voltStart >= VOLT_FAULT_TIME_MS) {
          tripSystem(vf);
        }
      }
    }
  }
  else {
    // ---------- Fault state: load stays isolated ----------
    if (AUTO_RETRY && (millis() - faultTime >= RETRY_DELAY_MS)) {
      bool isVoltFault = (systemState == OVERVOLT_FAULT || systemState == UNDERVOLT_FAULT);
      if (!isVoltFault || voltageInRange()) {
        systemState = NO_FAULT;
        overloadTiming = false;
        voltTiming = false;
        loadON();
      }
    }
  }

  // ---------- Refresh LCD / Serial ----------
  if (millis() - lastLcdUpdate >= 300) {
    lastLcdUpdate = millis();
    updateDisplay();

    Serial.print("V = ");
    Serial.print(voltageRMS, 1);
    Serial.print(" V   I = ");
    Serial.print(currentRMS, 2);
    Serial.print(" A   State: ");
    switch (systemState) {
      case NO_FAULT:        Serial.println("NORMAL");        break;
      case OVERLOAD_FAULT:  Serial.println("OVERLOAD");      break;
      case SHORT_FAULT:     Serial.println("SHORT CIRCUIT"); break;
      case OVERVOLT_FAULT:  Serial.println("OVER VOLTAGE");  break;
      case UNDERVOLT_FAULT: Serial.println("UNDER VOLTAGE"); break;
    }
  }
}

// =====================================================
// Reads current (A0) and voltage (A1) together and computes RMS values.
// The DC mid-point (offset) is removed automatically using the mean.
void readSensors() {
  unsigned long start = millis();
  double sumI = 0, sumI2 = 0, sumV = 0, sumV2 = 0;
  long n = 0;

  while (millis() - start < SAMPLE_WINDOW) {
    int ri = analogRead(CURRENT_PIN);
    int rv = analogRead(VOLTAGE_PIN);
    sumI  += ri;
    sumI2 += (double)ri * ri;
    sumV  += rv;
    sumV2 += (double)rv * rv;
    n++;
  }

  double meanI = sumI / n;
  double varI  = sumI2 / n - meanI * meanI;
  if (varI < 0) varI = 0;

  double meanV = sumV / n;
  double varV  = sumV2 / n - meanV * meanV;
  if (varV < 0) varV = 0;

  currentRMS = sqrt(varI) * ADC_REF / 1023.0 / SENSITIVITY;
  voltageRMS = sqrt(varV) * ADC_REF / 1023.0 * VOLT_CALIB;

  if (currentRMS < CURRENT_NOISE_FLOOR) currentRMS = 0.0;
  if (voltageRMS < VOLTAGE_NOISE_FLOOR) voltageRMS = 0.0;
}

// =====================================================
bool voltageInRange() {
  return (voltageRMS > UNDER_VOLTAGE_LIMIT + 5.0 &&
          voltageRMS < OVER_VOLTAGE_LIMIT - 5.0);
}

// =====================================================
void loadON() {
  digitalWrite(RELAY_PIN, RELAY_ON);
  setGreen(true);
  setRed(false);
  noTone(BUZZER_PIN);
  digitalWrite(BUZZER_PIN, LOW);
  relayOnTime = millis();
  lcd.clear();
}

// =====================================================
void tripSystem(Fault f) {
  digitalWrite(RELAY_PIN, RELAY_OFF);      // isolate the load immediately
  systemState = f;
  faultTime = millis();
  setGreen(false);
  setRed(true);
  tone(BUZZER_PIN, 2500);                  // continuous alarm
  lcd.clear();
}

// =====================================================
void setGreen(bool on) {
  digitalWrite(GREEN_LED_1, on ? HIGH : LOW);
  digitalWrite(GREEN_LED_2, on ? HIGH : LOW);
}

void setRed(bool on) {
  digitalWrite(RED_LED_1, on ? HIGH : LOW);
  digitalWrite(RED_LED_2, on ? HIGH : LOW);
}

// =====================================================
void updateDisplay() {
  char vStr[8], iStr[8], line[17];

  // Line 1 : voltage and current  e.g.  "V:230.5 I:0.43A "
  dtostrf(voltageRMS, 5, 1, vStr);
  dtostrf(currentRMS, 4, 2, iStr);
  snprintf(line, sizeof(line), "V:%s I:%sA", vStr, iStr);
  lcd.setCursor(0, 0);
  lcd.print(line);
  for (int i = strlen(line); i < 16; i++) lcd.print(' ');

  // Line 2 : status
  lcd.setCursor(0, 1);
  switch (systemState) {
    case NO_FAULT:        lcd.print("Status: NORMAL  "); break;
    case OVERLOAD_FAULT:  lcd.print("OVERLOAD-ISOLATE"); break;
    case SHORT_FAULT:     lcd.print("SHORT CKT-ISOLAT"); break;
    case OVERVOLT_FAULT:  lcd.print("OVER VOLT-ISOLAT"); break;
    case UNDERVOLT_FAULT: lcd.print("UNDER VOLT-ISOLA"); break;
  }
}
