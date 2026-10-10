#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <EEPROM.h>
#include <math.h>
#include <stdlib.h>

// ---------------- LCD ----------------
LiquidCrystal_I2C lcd(0x38, 20, 4);

// ---------------- KEYPAD ----------------
const byte ROWS = 4;
const byte COLS = 4;
char keys[ROWS][COLS] = {
  {'7', '8', '9', '/'},
  {'4', '5', '6', '*'},
  {'1', '2', '3', '-'},
  {'O', '0', '=', '+'}
};
byte rowPins[ROWS] = {22, 23, 24, 25};
byte colPins[COLS] = {26, 27, 28, 29};
Keypad keypad = Keypad(
  makeKeymap(keys), rowPins, colPins, ROWS, COLS
);
char numericInput[4] = "";
byte numericLength = 0;

// ---------------- ANALOG INPUTS ----------------
const byte voltagePins[3] = {A0, A1, A2};
const byte currentPins[3] = {A8, A9, A10};

// ---------------- RELAY OUTPUTS ----------------
// Mega D32 = PC5, D31 = PC6, D30 = PC7
const byte relayPins[3] = {32, 31, 30};
// Assumption: HIGH = TRIP, LOW = NORMAL
const byte RELAY_TRIP = LOW;
const byte RELAY_NORMAL = HIGH;

// ---------------- ADC / VOLTAGE CALIBRATION ----------------
const float ADC_REF = 5.0;
const float ADC_MAX = 1023.0;
const float DIVIDER_RATIO = 11.0;
const float TRANSFORMER_RATIO = 220.0 / 13.0;
const float VOLTAGE_CALIBRATION = 1.085;
const float ACS712_SENSITIVITY = 0.100;

// ---------------- MEASUREMENT ----------------
const unsigned long WINDOW_US = 100000UL; // 100 ms
const unsigned long KEY_SCAN_INTERVAL = 5;
unsigned long lastKeyScan = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastMeasurement = 0;
const unsigned long DISPLAY_INTERVAL_MS = 200;
const unsigned long MEASUREMENT_INTERVAL_MS = 50;

// ---------------- EEPROM SETTINGS ----------------
const int EEPROM_ADDR = 0;
const uint16_t EEPROM_MAGIC = 0x3A71;
struct ProtectionSettings {
  uint16_t magic;
  int rOV;
  int rUV;
  int yOV;
  int yUV;
  int bOV;
  int bUV;
  int rOC;
  int yOC;
  int bOC;
};

ProtectionSettings settings;

void setDefaultSettings() {
  settings.magic = EEPROM_MAGIC;
  settings.rOV = 250;
  settings.rUV = 180;
  settings.yOV = 250;
  settings.yUV = 180;
  settings.bOV = 250;
  settings.bUV = 180;
  settings.rOC = 10;
  settings.yOC = 10;
  settings.bOC = 10;
}
void loadSettings() {
  EEPROM.get(EEPROM_ADDR, settings);
  bool valid = settings.magic == EEPROM_MAGIC;
  valid &= settings.rOV >= 190 && settings.rOV <= 280;
  valid &= settings.rUV >= 100 && settings.rUV <= 240;
  valid &= settings.yOV >= 190 && settings.yOV <= 280;
  valid &= settings.yUV >= 100 && settings.yUV <= 240;
  valid &= settings.bOV >= 190 && settings.bOV <= 280;
  valid &= settings.bUV >= 100 && settings.bUV <= 240;
  valid &= settings.rOC >= 1 && settings.rOC <= 20;
  valid &= settings.yOC >= 1 && settings.yOC <= 20;
  valid &= settings.bOC >= 1 && settings.bOC <= 20;
  valid &= settings.rUV < settings.rOV;
  valid &= settings.yUV < settings.yOV;
  valid &= settings.bUV < settings.bOV;
  if (!valid) {
    setDefaultSettings();
    EEPROM.put(EEPROM_ADDR, settings);
  }
}
void saveSettings() {
  settings.magic = EEPROM_MAGIC;
  EEPROM.put(EEPROM_ADDR, settings);
}

// ---------------- MEASURED VALUES ----------------
float phaseVoltage[3] = {0, 0, 0};
float phaseCurrent[3] = {0, 0, 0};
bool voltageValid[3] = {false, false, false};
bool currentValid[3] = {false, false, false};
bool voltageFault[3] = {false, false, false};
bool overCurrentFault[3] = {false, false, false};
bool settingsMode = false;
byte selectedSetting = 0;
const byte TOTAL_SETTINGS = 9;

// ---------------- FAULT STATUS ----------------
enum FaultCode {
  FAULT_NONE,
  FAULT_SENSOR,
  FAULT_PHASE_R_OV,
  FAULT_PHASE_R_UV,
  FAULT_PHASE_Y_OV,
  FAULT_PHASE_Y_UV,
  FAULT_PHASE_B_OV,
  FAULT_PHASE_B_UV,
  FAULT_R_OC,
  FAULT_Y_OC,
  FAULT_B_OC
};
FaultCode phaseFault[3] = {
  FAULT_NONE,
  FAULT_NONE,
  FAULT_NONE
};
FaultCode systemFault = FAULT_NONE;

// ---------------- FORWARD DECLARATIONS ----------------
void openSettings();
void handleSettingsKey(char key);
void scanKeypad();
void updateDisplay();
void updateFaults();
float measureRMS(byte pin, bool &valid, bool isCurrentSensor);


// ------------------KEYPAD SCANNING-------------------
void scanKeypad() {
  char key = keypad.getKey();
  if (key == NO_KEY) return;
  if (settingsMode) {
    handleSettingsKey(key);
  } else if (key == '*') {
    openSettings();
  }
}

// ------------------------RMS MEASUREMENT-----------------
float measureRMS(byte pin, bool &valid, bool isCurrentSensor) {
  unsigned long start = micros();
  double sum = 0.0;
  double sumSquares = 0.0;
  unsigned long count = 0;
  unsigned int clipped = 0;
  while ((unsigned long)(micros() - start) < WINDOW_US) {
    if (millis() - lastKeyScan >= KEY_SCAN_INTERVAL) {
      lastKeyScan = millis();
      scanKeypad();
    }
    int raw = analogRead(pin);
    float sampleV = raw * ADC_REF / ADC_MAX;
    if (raw <= 2 || raw >= 1021) {
      clipped++;
    }
    sum += sampleV;
    sumSquares += (double)sampleV * sampleV;
    count++;
  }
  if (count < 100 || clipped > count / 100UL) {
    valid = false;
    return 0.0;
  }
  double mean = sum / count;
  double variance = sumSquares / count - mean * mean;
  if (variance < 0) variance = 0;
  float acRMS = sqrt(variance);
  if (isCurrentSensor) {
    if (mean < 1.0 || mean > 4.0) {
      valid = false;
      return 0.0;
    }
    valid = true;
    return acRMS / ACS712_SENSITIVITY;
  }
  if (acRMS < 0.005 || acRMS > 1.75) {
    valid = false;
    return 0.0;
  }
  valid = true;
  return acRMS * DIVIDER_RATIO *
         TRANSFORMER_RATIO * VOLTAGE_CALIBRATION;
}


// --------------------READ ALL PHASES--------------------
void readMeasurements() {
  for (byte i = 0; i < 3; i++) {
    phaseVoltage[i] =
      measureRMS(voltagePins[i], voltageValid[i], false);
    phaseCurrent[i] =
      measureRMS(currentPins[i], currentValid[i], true);
  }
}


// ---------------GET SELECTED SETTING----------------------
int *getSettingPointer(byte index) {
  switch (index) {
    case 0: return &settings.rOV;
    case 1: return &settings.rUV;
    case 2: return &settings.yOV;
    case 3: return &settings.yUV;
    case 4: return &settings.bOV;
    case 5: return &settings.bUV;
    case 6: return &settings.rOC;
    case 7: return &settings.yOC;
    case 8: return &settings.bOC;
  }
  return &settings.rOV;
}
const char *getSettingName(byte index) {
  switch (index) {
    case 0: return "R Over Voltage";
    case 1: return "R Under Voltage";
    case 2: return "Y Over Voltage";
    case 3: return "Y Under Voltage";
    case 4: return "B Over Voltage";
    case 5: return "B Under Voltage";
    case 6: return "R Over Current";
    case 7: return "Y Over Current";
    case 8: return "B Over Current";
  }
  return "Setting";
}


// --------------------SETTINGS MENU------------------------
void openSettings() {
  settingsMode = true;
  selectedSetting = 0;
  lcd.clear();
  updateDisplay();
}
void handleSettingsKey(char key) {
  // O clears typed digits; if no digits are pending, exit without saving.
  if (key == 'O') {
    if (numericLength > 0) {
      numericLength = 0;
      numericInput[0] = '\0';
      updateDisplay();
    } else {
      loadSettings();
      settingsMode = false;
      lcd.clear();
    }
    return;
  }
  // Buffer numeric input (up to 3 digits).
  if (key >= '0' && key <= '9') {
    if (numericLength < sizeof(numericInput) - 1) {
      numericInput[numericLength++] = key;
      numericInput[numericLength] = '\0';
      updateDisplay();
    }
    return;
  }
  // Move to the next setting.
  if (key == '/') {
    selectedSetting++;
    if (selectedSetting >= TOTAL_SETTINGS) {
      selectedSetting = 0;
    }
    numericLength = 0;
    numericInput[0] = '\0';
    updateDisplay();
    return;
  }
  int *value = getSettingPointer(selectedSetting);
  // '=' applies any typed number, saves settings, and exits the menu.
  if (key == '=') {
    if (numericLength > 0) {
      int enteredValue = atoi(numericInput);
      if (selectedSetting <= 5) {
        if (selectedSetting % 2 == 0) {
          enteredValue = constrain(enteredValue, 190, 280);
        } else {
          enteredValue = constrain(enteredValue, 100, 240);
        }
      } else {
        enteredValue = constrain(enteredValue, 1, 20);
      }
      *value = enteredValue;
      // Keep UV below OV for the same phase.
      if (selectedSetting <= 5) {
        byte ovIndex = (selectedSetting / 2) * 2;
        int *ovValue = getSettingPointer(ovIndex);
        int *uvValue = getSettingPointer(ovIndex + 1);
        if (*uvValue >= *ovValue) {
          if (selectedSetting == ovIndex) {
            *uvValue = *ovValue - 1;
          } else {
            *ovValue = *uvValue + 1;
          }
        }
      }
      numericLength = 0;
      numericInput[0] = '\0';
    }
    saveSettings();
    settingsMode = false;
    lcd.clear();
    return;
  }
  // +/- changes the stored value by one.
  if (key == '+' || key == '-') {
    numericLength = 0;
    numericInput[0] = '\0';
    if (key == '+') {
      (*value)++;
    } else {
      (*value)--;
    }
    if (selectedSetting <= 5) {
      if (selectedSetting % 2 == 0) {
        *value = constrain(*value, 190, 280);
      } else {
        *value = constrain(*value, 100, 240);
      }
      byte ovIndex = (selectedSetting / 2) * 2;
      int *ovValue = getSettingPointer(ovIndex);
      int *uvValue = getSettingPointer(ovIndex + 1);
      if (*uvValue >= *ovValue) {
        if (selectedSetting == ovIndex) {
          *uvValue = *ovValue - 1;
        } else {
          *ovValue = *uvValue + 1;
        }
      }
    } else {
      *value = constrain(*value, 1, 20);
    }
    updateDisplay();
  }
}

// ------------------------FAULT DETECTION-------------------------
void updateFaults() {
  systemFault = FAULT_NONE;
  int ovLimits[3] = {
    settings.rOV, settings.yOV, settings.bOV
  };
  int uvLimits[3] = {
    settings.rUV, settings.yUV, settings.bUV
  };
  int ocLimits[3] = {
    settings.rOC, settings.yOC, settings.bOC
  };
  FaultCode ovFaults[3] = {
    FAULT_PHASE_R_OV, FAULT_PHASE_Y_OV, FAULT_PHASE_B_OV
  };
  FaultCode uvFaults[3] = {
    FAULT_PHASE_R_UV, FAULT_PHASE_Y_UV, FAULT_PHASE_B_UV
  };
  FaultCode ocFaults[3] = {
    FAULT_R_OC, FAULT_Y_OC, FAULT_B_OC
  };
  
  for (byte i = 0; i < 3; i++) {
    voltageFault[i] = false;
    overCurrentFault[i] = false;
    phaseFault[i] = FAULT_NONE;
    if (!voltageValid[i] || !currentValid[i]) {
      phaseFault[i] = FAULT_SENSOR;
    } else if (phaseVoltage[i] > ovLimits[i]) {
      voltageFault[i] = true;
      phaseFault[i] = ovFaults[i];
    } else if (phaseVoltage[i] < uvLimits[i]) {
      voltageFault[i] = true;
      phaseFault[i] = uvFaults[i];
    } else if (phaseCurrent[i] >= ocLimits[i]) {
      overCurrentFault[i] = true;
      phaseFault[i] = ocFaults[i];
    }
    
    if (systemFault == FAULT_NONE && phaseFault[i] != FAULT_NONE) {
      systemFault = phaseFault[i];
    }
  }
}

// -------------------------RELAY CONTROL---------------------------
void updateRelays() {
  // Each relay responds only to its own phase fault.
  for (byte i = 0; i < 3; i++) {
    if (phaseFault[i] != FAULT_NONE) {
      digitalWrite(relayPins[i], RELAY_TRIP);
    } else {
      digitalWrite(relayPins[i], RELAY_NORMAL);
    }
  }
}


// --------------------------LCD DISPLAY------------------------------
void updateDisplay() {
  if (settingsMode) {
    int *value = getSettingPointer(selectedSetting);
    lcd.setCursor(0, 0);
    lcd.print("SETTINGS ");
    lcd.print(selectedSetting + 1);
    lcd.print("/");
    lcd.print(TOTAL_SETTINGS);
    lcd.print("       ");
    lcd.setCursor(0, 1);
    lcd.print(getSettingName(selectedSetting));
    lcd.print("      ");
    lcd.setCursor(0, 2);
    lcd.print("Value: ");
    if (numericLength > 0) {
      lcd.print(numericInput);
    } else {
      lcd.print(*value);
    }
    lcd.print("          ");
    lcd.setCursor(0, 3);
    lcd.print("/ Next +/- Adj =Save");
    lcd.print(" ");
    return;
  }
  // Row 1: R phase
  lcd.setCursor(0, 0);
  lcd.print("R:");
  if (voltageValid[0]) {
    lcd.print(phaseVoltage[0], 0);
    lcd.print("V");
  } else {
    lcd.print("---V");
  }
  lcd.print(" I:");
  if (currentValid[0]) {
    lcd.print(phaseCurrent[0], 1);
    lcd.print("A ");
  } else {
    lcd.print("---A ");
  }
  lcd.setCursor(16, 0);
  lcd.print("    ");
  lcd.setCursor(16, 0);
  switch (phaseFault[0]) {
    case FAULT_NONE:       lcd.print("Norm"); break;
    case FAULT_SENSOR:     lcd.print("Err "); break;
    case FAULT_PHASE_R_OV: lcd.print("OV  "); break;
    case FAULT_PHASE_R_UV: lcd.print("UV  "); break;
    case FAULT_R_OC:       lcd.print("OC  "); break;
    default:               lcd.print("Norm"); break;
  }
  // Row 2: Y phase
  lcd.setCursor(0, 1);
  lcd.print("Y:");
  if (voltageValid[1]) {
    lcd.print(phaseVoltage[1], 0);
    lcd.print("V");
  } else {
    lcd.print("---V");
  }
  lcd.print(" I:");
  if (currentValid[1]) {
    lcd.print(phaseCurrent[1], 1);
    lcd.print("A ");
  } else {
    lcd.print("---A ");
  }
  lcd.setCursor(16, 1);
  lcd.print("    ");
  lcd.setCursor(16, 1);
  switch (phaseFault[1]) {
    case FAULT_NONE:       lcd.print("Norm"); break;
    case FAULT_SENSOR:     lcd.print("Err "); break;
    case FAULT_PHASE_Y_OV: lcd.print("OV  "); break;
    case FAULT_PHASE_Y_UV: lcd.print("UV  "); break;
    case FAULT_Y_OC:       lcd.print("OC  "); break;
    default:               lcd.print("Norm"); break;
  }
  // Row 3: B phase
  lcd.setCursor(0, 2);
  lcd.print("B:");
  if (voltageValid[2]) {
    lcd.print(phaseVoltage[2], 0);
    lcd.print("V");
  } else {
    lcd.print("---V");
  }
  lcd.print(" I:");
  if (currentValid[2]) {
    lcd.print(phaseCurrent[2], 1);
    lcd.print("A ");
  } else {
    lcd.print("---A ");
  }
  lcd.setCursor(16, 2);
  lcd.print("    ");
  lcd.setCursor(16, 2);
  switch (phaseFault[2]) {
    case FAULT_NONE:       lcd.print("Norm"); break;
    case FAULT_SENSOR:     lcd.print("Err "); break;
    case FAULT_PHASE_B_OV: lcd.print("OV  "); break;
    case FAULT_PHASE_B_UV: lcd.print("UV  "); break;
    case FAULT_B_OC:       lcd.print("OC  "); break;
    default:               lcd.print("Norm"); break;
  }
}

// -------------------------SETUP------------------------
const char* faultText(byte fault) {
  switch (fault) {
    case FAULT_NONE:       return "NORMAL";
    case FAULT_SENSOR:     return "SENSOR_ERROR";
    case FAULT_PHASE_R_OV:
    case FAULT_PHASE_Y_OV:
    case FAULT_PHASE_B_OV: return "OVERVOLTAGE";
    case FAULT_PHASE_R_UV:
    case FAULT_PHASE_Y_UV:
    case FAULT_PHASE_B_UV: return "UNDERVOLTAGE";
    case FAULT_R_OC:
    case FAULT_Y_OC:
    case FAULT_B_OC:       return "OVERCURRENT";
    default:               return "UNKNOWN";
  }
}
void setup() {
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  for (byte i = 0; i < 3; i++) {
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], RELAY_TRIP);
  }
  loadSettings();
  lcd.setCursor(0, 0);
  lcd.print("3-PHASE PROTECTION");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");
  delay(1000);
  readMeasurements();
  updateFaults();
  updateRelays();
  lcd.clear();
}


// -------------------------MAIN LOOP-------------------------
void loop() {
  // Keypad remains responsive during RMS measurement too.
  if (millis() - lastKeyScan >= KEY_SCAN_INTERVAL) {
    lastKeyScan = millis();
    scanKeypad();
  }
  if (settingsMode) {
    updateDisplay();
    return;
  }
  if (millis() - lastMeasurement >= MEASUREMENT_INTERVAL_MS) {
    lastMeasurement = millis();
    readMeasurements();
    updateFaults();
    updateRelays();
    Serial.print("R: V=");
      Serial.print(phaseVoltage[0], 1);
      Serial.print("V I=");
      Serial.print(phaseCurrent[0], 2);
      Serial.print("A STATUS=");
      Serial.print(faultText((byte)phaseFault[0]));
      Serial.print(" | Y: V=");
      Serial.print(phaseVoltage[1], 1);
      Serial.print("V I=");
      Serial.print(phaseCurrent[1], 2);
      Serial.print("A STATUS=");
      Serial.print(faultText((byte)phaseFault[1]));
      Serial.print(" | B: V=");
      Serial.print(phaseVoltage[2], 1);
      Serial.print("V I=");
      Serial.print(phaseCurrent[2], 2);
      Serial.print("A STATUS=");
      Serial.println(faultText((byte)phaseFault[2]));
  }
  if (millis() - lastDisplayUpdate >= DISPLAY_INTERVAL_MS) {
    lastDisplayUpdate = millis();
    updateDisplay();
  }
}
