constexpr unsigned int SENSOR_INTEGRATION_MS = 100; // Per spectral group, 1-600 ms.

#include <Arduino.h>
#include <Wire.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>

// ============================================
// 24CH PWM + 4 AS7341 Sensors
// Teensy 4.1 / PCA9548A / AL8862
// ============================================

// ---------- PWM ----------
const uint8_t PWM_COUNT = 24;
const uint32_t PWM_FREQ = 200;
const uint16_t PWM_MAX = 4095; // 12-bit

// LED channel 1 through 24, in the same order as the PWM list.
const uint8_t pwmPins[PWM_COUNT] = {
   0,  1,  2,  3,  4,  5,
   6,  7,  8,  9, 10, 11,
  12, 36, 14, 15, 18, 19,
  22, 33, 24, 25, 28, 29
};

float pwmDuty[PWM_COUNT] = {0};

// Software interlock:
// PWM commands work only after ARM.
bool pwmArmed = false;

// ---------- AS7341 ----------
const uint8_t MUX_ADDR = 0x70;
const uint8_t SENSOR_ADDR = 0x39;
// Sensor IDs are indices into this PCA9548A channel map.
constexpr uint8_t SENSOR_MUX_CHANNELS[] = {0, 1, 2, 3};
constexpr uint8_t SENSOR_COUNT = sizeof(SENSOR_MUX_CHANNELS);

constexpr uint8_t AS_ENABLE = 0x80;
constexpr uint8_t AS_ATIME = 0x81;
constexpr uint8_t AS_WTIME = 0x83;
constexpr uint8_t AS_ASTATUS = 0x94;
constexpr uint8_t AS_STATUS2 = 0xA3;
constexpr uint8_t AS_CFG0 = 0xA9;
constexpr uint8_t AS_GAIN = 0xAA;
constexpr uint8_t AS_CFG6 = 0xAF;
constexpr uint8_t AS_ASTEP_LOW = 0xCA;
constexpr uint8_t AS_ASTEP_HIGH = 0xCB;
constexpr uint8_t AS_CONFIG = 0x70;
constexpr uint8_t AS_LED = 0x74;
constexpr uint8_t AS_POWER = 0x01;
constexpr uint8_t AS_SPECTRAL_ENABLE = 0x02;
constexpr uint8_t AS_WAIT_ENABLE = 0x08;
constexpr uint8_t AS_SMUX_ENABLE = 0x10;
constexpr uint8_t AS_DATA_VALID = 0x40;
constexpr uint8_t AS_BANK_SELECT = 0x10;
constexpr uint8_t AS_WAIT_LONG = 0x04;
constexpr uint8_t AS_SMUX_WRITE = 0x10;

// Integration = (ATIME + 1) * (ASTEP + 1) * 2.78 us.
static_assert(SENSOR_INTEGRATION_MS >= 1 && SENSOR_INTEGRATION_MS <= 600,
              "SENSOR_INTEGRATION_MS must be 1-600");
constexpr uint8_t AS_ATIME_VALUE = 5;
constexpr uint32_t AS_STEP_NS = (AS_ATIME_VALUE + 1UL) * 2780UL;
constexpr uint16_t AS_ASTEP_VALUE =
    (SENSOR_INTEGRATION_MS * 1000000UL + AS_STEP_NS / 2) / AS_STEP_NS - 1;
constexpr uint32_t AS_ADC_FULL_SCALE =
    (AS_ATIME_VALUE + 1UL) * (AS_ASTEP_VALUE + 1UL);
constexpr uint32_t AS_ADC_MAX_COUNT =
    AS_ADC_FULL_SCALE < UINT16_MAX ? AS_ADC_FULL_SCALE : UINT16_MAX;
// Allow time to read and stop the ADC before the next measurement.
constexpr uint32_t AS_WAIT_TICK_US = 2780;
constexpr uint8_t AS_WTIME_VALUE =
    ((SENSOR_INTEGRATION_MS + 90UL) * 1000UL + AS_WAIT_TICK_US - 1) /
        AS_WAIT_TICK_US - 1;
constexpr uint32_t SMUX_TIMEOUT_MS = 20;
constexpr uint32_t MEASUREMENT_TIMEOUT_MS = SENSOR_INTEGRATION_MS + 140UL;

// SMUX routes from https://github.com/DFRobot/DFRobot_AS7341.
// ADC0-3 = F1-F4 or F5-F8, ADC4 = Clear, ADC5 = NIR.
constexpr uint8_t SMUX_F1_F4[] = {
  0x30, 0x01, 0x00, 0x00, 0x00, 0x42, 0x00, 0x00, 0x50, 0x00,
  0x00, 0x00, 0x20, 0x04, 0x00, 0x30, 0x01, 0x50, 0x00, 0x06
};
constexpr uint8_t SMUX_F5_F8[] = {
  0x00, 0x00, 0x00, 0x40, 0x02, 0x00, 0x10, 0x03, 0x50, 0x10,
  0x03, 0x00, 0x00, 0x00, 0x24, 0x00, 0x00, 0x50, 0x00, 0x06
};

// ============================================
// PWM Functions
// ============================================

void setPWM(uint8_t channel, float duty) {
  if (channel >= PWM_COUNT) return;
  if (!isfinite(duty)) return;
  duty = roundf(constrain(duty, 0.0f, 100.0f) * 10.0f) / 10.0f;

  pwmDuty[channel] = duty;

  // Use steady GPIO levels at the two endpoints.
  if (duty == 0.0f || duty == 100.0f) {
    digitalWrite(pwmPins[channel], duty == 0.0f ? LOW : HIGH);
    pinMode(pwmPins[channel], OUTPUT);
    return;
  }

  uint16_t value =
      static_cast<uint16_t>(lroundf(duty * PWM_MAX / 100.0f));

  analogWrite(pwmPins[channel], value);
}

void allPWM(float duty) {
  for (uint8_t i = 0; i < PWM_COUNT; i++) {
    setPWM(i, duty);
  }
}

void pwmOFF() {
  allPWM(0);
  pwmArmed = false;
  Serial.println("ALL PWM OFF / DISARMED");
}

void initPWM() {
  // Establish LOW before enabling the outputs.
  for (uint8_t i = 0; i < PWM_COUNT; i++) {
    digitalWrite(pwmPins[i], LOW);
    pinMode(pwmPins[i], OUTPUT);
  }

  analogWriteResolution(12);

  for (uint8_t i = 0; i < PWM_COUNT; i++) {
    analogWriteFrequency(pwmPins[i], PWM_FREQ);
    setPWM(i, 0);
  }

  pwmArmed = false;
}

// ============================================
// PCA9548A Functions
// ============================================

bool selectMux(uint8_t channel) {
  if (channel > 7) return false;

  Wire1.beginTransmission(MUX_ADDR);
  Wire1.write(1 << channel);

  return Wire1.endTransmission() == 0;
}

bool deviceFound(uint8_t addr) {
  Wire1.beginTransmission(addr);
  return Wire1.endTransmission() == 0;
}

bool disableMux() {
  Wire1.beginTransmission(MUX_ADDR);
  Wire1.write(0);
  return Wire1.endTransmission() == 0;
}

// ============================================
// AS7341 Functions
// ============================================

bool writeSensorRegister(uint8_t reg, uint8_t value) {
  Wire1.beginTransmission(SENSOR_ADDR);
  Wire1.write(reg);
  Wire1.write(value);
  return Wire1.endTransmission() == 0;
}

bool readSensorRegisters(uint8_t reg, uint8_t *data, uint8_t count) {
  Wire1.beginTransmission(SENSOR_ADDR);
  Wire1.write(reg);
  if (Wire1.endTransmission(false) != 0) return false;
  if (Wire1.requestFrom(SENSOR_ADDR, count) != count) return false;
  for (uint8_t i = 0; i < count; i++) {
    data[i] = Wire1.read();
  }
  return true;
}

bool clearSensorBits(uint8_t reg, uint8_t mask) {
  uint8_t value;
  return readSensorRegisters(reg, &value, 1) &&
         writeSensorRegister(reg, value & ~mask);
}

bool initSensor() {
  // Keep the ADC stopped while configuring the selected sensor.
  if (!writeSensorRegister(AS_ENABLE, AS_POWER)) return false;
  delay(2);

  uint8_t cfg0;
  if (!readSensorRegisters(AS_CFG0, &cfg0, 1) ||
      !writeSensorRegister(AS_CFG0, cfg0 | AS_BANK_SELECT)) {
    return false;
  }

  // Set SPM mode and disable the sensor's illumination LED.
  const bool configured = clearSensorBits(AS_CONFIG, 0x0B) &&
                          clearSensorBits(AS_LED, 0x80);
  const bool bankRestored =
      writeSensorRegister(AS_CFG0, cfg0 & ~(AS_BANK_SELECT | AS_WAIT_LONG));

  return configured && bankRestored &&
         writeSensorRegister(AS_ATIME, AS_ATIME_VALUE) &&
         writeSensorRegister(AS_ASTEP_LOW, AS_ASTEP_VALUE & 0xFF) &&
         writeSensorRegister(AS_ASTEP_HIGH, AS_ASTEP_VALUE >> 8) &&
         writeSensorRegister(AS_GAIN, 0) && // 0.5x gain
         writeSensorRegister(AS_WTIME, AS_WTIME_VALUE);
}

const char *waitForSensorFlag(uint8_t reg, uint8_t mask, uint8_t expected,
                              uint32_t timeoutMs, const char *timeoutError) {
  const uint32_t start = millis();
  do {
    uint8_t status;
    if (!readSensorRegisters(reg, &status, 1)) return "SENSOR_I2C_ERROR";
    if ((status & mask) == expected) return nullptr;
    delay(1);
  } while (millis() - start < timeoutMs);
  return timeoutError;
}

const char *readSpectralGroup(const uint8_t (&smux)[20], uint16_t (&values)[5]) {
  if (!writeSensorRegister(AS_CFG6, AS_SMUX_WRITE)) return "SENSOR_I2C_ERROR";
  for (uint8_t reg = 0; reg < sizeof(smux); reg++) {
    if (!writeSensorRegister(reg, smux[reg])) return "SENSOR_I2C_ERROR";
  }
  if (!writeSensorRegister(AS_ENABLE, AS_POWER | AS_SMUX_ENABLE)) {
    return "SENSOR_I2C_ERROR";
  }
  const char *error = waitForSensorFlag(AS_ENABLE, AS_SMUX_ENABLE, 0,
                                       SMUX_TIMEOUT_MS, "SMUX_TIMEOUT");
  if (error) return error;

  uint8_t data[13]; // ASTATUS latches all six 16-bit ADC results.
  if (writeSensorRegister(AS_ENABLE, AS_POWER | AS_SPECTRAL_ENABLE | AS_WAIT_ENABLE)) {
    error = waitForSensorFlag(AS_STATUS2, AS_DATA_VALID, AS_DATA_VALID,
                              MEASUREMENT_TIMEOUT_MS, "MEAS_TIMEOUT");
    if (!error && !readSensorRegisters(AS_ASTATUS, data, sizeof(data))) {
      error = "SENSOR_I2C_ERROR";
    }
  } else {
    error = "SENSOR_I2C_ERROR";
  }

  // Stop integration before switching the SMUX or leaving this mux channel.
  if (!writeSensorRegister(AS_ENABLE, AS_POWER)) return "SENSOR_I2C_ERROR";
  if (error) return error;

  if (data[0] & 0x80) return "SATURATED"; // ASTATUS.ASAT_STATUS
  for (uint8_t adc = 0; adc < 5; adc++) {
    const uint8_t low = 1 + adc * 2;
    values[adc] = (static_cast<uint16_t>(data[low + 1]) << 8) | data[low];
    if (values[adc] >= AS_ADC_MAX_COUNT) return "SATURATED";
  }
  return nullptr;
}

void printCsvError(uint32_t runId, unsigned int sensor, const char *error) {
  Serial.printf("CSV_ERROR,%lu,%u,%s\n", (unsigned long)runId, sensor, error);
}

bool readSensor(uint8_t ch, uint32_t runId) {
  const unsigned int sensor = ch;
  if (!selectMux(SENSOR_MUX_CHANNELS[ch])) {
    printCsvError(runId, sensor, "MUX_ERROR");
    return false;
  }
  if (!deviceFound(SENSOR_ADDR)) {
    printCsvError(runId, sensor, "DISCONNECTED");
    return false;
  }
  // Reapply settings after a possible sensor reset or reconnection.
  if (!initSensor()) {
    printCsvError(runId, sensor, "NOT_READY");
    return false;
  }

  uint16_t first[5], second[5];
  const char *error = readSpectralGroup(SMUX_F1_F4, first);
  if (!error) error = readSpectralGroup(SMUX_F5_F8, second);
  if (error) {
    printCsvError(runId, sensor, error);
    return false;
  }

  // CSV,run_id,uptime_ms,sensor,F1,F2,F3,F4,F5,F6,F7,F8,Clear
  // As before, Clear comes from ADC4 in the second integration.
  Serial.printf(
    "CSV,%lu,%lu,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n",
    (unsigned long)runId, (unsigned long)millis(), sensor,
    (unsigned int)first[0], (unsigned int)first[1],
    (unsigned int)first[2], (unsigned int)first[3],
    (unsigned int)second[0], (unsigned int)second[1],
    (unsigned int)second[2], (unsigned int)second[3],
    (unsigned int)second[4]
  );
  return true;
}

uint32_t measurementRun = 0;

// endChannel is exclusive. Both forms of READ use the same CSV format.
void measureSensors(uint8_t firstChannel, uint8_t endChannel) {
  const uint32_t runId = ++measurementRun;
  Serial.printf("CSV_BEGIN,%lu,%lu\n", (unsigned long)runId, (unsigned long)millis());

  unsigned int successCount = 0;
  if (!deviceFound(MUX_ADDR)) {
    for (uint8_t ch = firstChannel; ch < endChannel; ch++) {
      printCsvError(runId, ch, "MUX_NOT_FOUND");
    }
  } else {
    for (uint8_t ch = firstChannel; ch < endChannel; ch++) {
      if (readSensor(ch, runId)) successCount++;
    }
    if (!disableMux()) printCsvError(runId, firstChannel, "MUX_DISABLE_ERROR");
  }

  Serial.printf("CSV_END,%lu,%u\n", (unsigned long)runId, successCount);
}

// ============================================
// Serial Commands
// ============================================

void showStatus() {
  Serial.println();
  Serial.println("===== PWM STATUS =====");

  Serial.printf("ARMED: %s\n",
                pwmArmed ? "YES" : "NO");
  Serial.printf("SENSOR INTEGRATION: %u ms per group\n", SENSOR_INTEGRATION_MS);

  for (uint8_t i = 0; i < PWM_COUNT; i++) {
    Serial.printf(
      "CH%02d / GPIO%02d : %5.1f%%\n",
      i + 1, pwmPins[i], pwmDuty[i]
    );
  }
}

void printHelp() {
  Serial.println();
  Serial.println("===== COMMANDS =====");
  Serial.println("HELP       : Show commands");
  Serial.println("STATUS     : Show PWM status");
  Serial.println("ARM        : Enable PWM commands");
  Serial.println("OFF        : All PWM OFF and disarm");
  Serial.println("SET 1 50   : Channel 1 to 50%");
  Serial.println("ALL 50     : All channels to 50%");
  Serial.println("[v1,...,v24] : Set channels 1-24 to 24 duties (0-100%)");
  Serial.printf("READ       : Read all %u sensors\n", (unsigned int)SENSOR_COUNT);
  Serial.printf("READ 0     : Read one sensor (IDs 0-%u)\n", (unsigned int)(SENSOR_COUNT - 1));
  Serial.println("====================");
}

bool parsePwmList(const char *input, float (&duties)[PWM_COUNT]) {
  // The caller checks the leading '[' and trims the command.
  const char *cursor = input + 1;
  for (uint8_t i = 0; i < PWM_COUNT; i++) {
    char *end;
    const double duty = strtod(cursor, &end);
    if (end == cursor || !isfinite(duty) || duty < 0.0 || duty > 100.0) {
      return false;
    }

    while (isspace(static_cast<unsigned char>(*end))) end++;
    const char separator = i + 1 == PWM_COUNT ? ']' : ',';
    if (*end != separator) return false;
    duties[i] = static_cast<float>(duty);
    cursor = end + 1;
  }
  return *cursor == '\0';
}

void handleCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  if (cmd == "HELP") {
    printHelp();
    return;
  }

  if (cmd == "STATUS") {
    showStatus();
    return;
  }

  if (cmd == "OFF") {
    pwmOFF();
    return;
  }

  if (cmd == "READ") {
    measureSensors(0, SENSOR_COUNT);
    return;
  }

  if (cmd.startsWith("READ ")) {
    String sensor = cmd.substring(5);
    sensor.trim();
    if (sensor.length() != 1 || sensor[0] < '0' ||
        sensor[0] >= '0' + SENSOR_COUNT) {
      Serial.printf("ERROR: Use READ with a sensor ID from 0 to %u\n",
                    (unsigned int)(SENSOR_COUNT - 1));
      return;
    }
    const uint8_t channel = sensor[0] - '0';
    measureSensors(channel, channel + 1);
    return;
  }

  if (cmd == "ARM") {
    pwmArmed = true;
    Serial.println("PWM ARMED");
    return;
  }

  if (cmd.startsWith("[")) {
    if (!pwmArmed) {
      Serial.println("ERROR: Send ARM first");
      return;
    }
    float duties[PWM_COUNT];
    if (!parsePwmList(cmd.c_str(), duties)) {
      Serial.println("ERROR: Expected [v1,...,v24] with 24 duties from 0 to 100");
      return;
    }
    // Apply the list only after every element has passed validation.
    for (uint8_t i = 0; i < PWM_COUNT; i++) {
      setPWM(i, duties[i]);
    }
    Serial.println("OK PWM 24");
    return;
  }

  int channel;
  float duty;

  if (sscanf(cmd.c_str(),
             "SET %d %f",
             &channel, &duty) == 2) {

    if (!pwmArmed) {
      Serial.println("ERROR: Send ARM first");
      return;
    }

    if (channel < 1 || channel > 24 ||
        !isfinite(duty) || duty < 0 || duty > 100) {
      Serial.println("ERROR: Invalid channel/duty");
      return;
    }

    setPWM(channel - 1, duty);

    Serial.printf(
      "CH%d = %.1f%%\n",
      channel, pwmDuty[channel - 1]
    );
    return;
  }

  if (sscanf(cmd.c_str(),
             "ALL %f", &duty) == 1) {

    if (!pwmArmed) {
      Serial.println("ERROR: Send ARM first");
      return;
    }

    if (!isfinite(duty) || duty < 0 || duty > 100) {
      Serial.println("ERROR: Duty must be 0-100");
      return;
    }

    allPWM(duty);
    Serial.printf("ALL PWM = %.1f%%\n", pwmDuty[0]);
    return;
  }

  if (cmd.length() > 0) {
    Serial.println("UNKNOWN COMMAND");
    printHelp();
  }
}

// ============================================
// SETUP
// ============================================

void setup() {
  // Initialize all PWM outputs to OFF first.
  initPWM();

  Serial.begin(115200);

  Wire1.begin();
  Wire1.setClock(100000);

  delay(2000);

  Serial.println();
  Serial.println("=== PHOTOREACTOR CONTROLLER ===");
  Serial.printf("24CH PWM + %u AS7341\n", (unsigned int)SENSOR_COUNT);

  if (deviceFound(MUX_ADDR)) {
    Serial.println("PCA9548A FOUND");

    for (uint8_t ch = 0; ch < SENSOR_COUNT; ch++) {
      const bool ready = selectMux(SENSOR_MUX_CHANNELS[ch]) && initSensor();

      Serial.printf(
        "Sensor %d: %s\n",
        ch,
        ready ? "INIT OK" : "INIT FAILED"
      );
    }

    disableMux();
  } else {
    Serial.println("PCA9548A NOT FOUND");
  }

  Serial.println("PWM DEFAULT: ALL OFF");
  printHelp();
}

// ============================================
// LOOP
// ============================================


void loop() {
  static char line[512];
  static size_t length = 0;
  static bool discardLine = false;

  while (Serial.available() > 0) {
    const int byte = Serial.read();
    if (byte < 0) return;

    if (byte == '\n') {
      if (discardLine) {
        Serial.println("ERROR: Invalid serial line");
      } else if (length != 0) {
        line[length] = '\0';
        handleCommand(String(line));
      }
      length = 0;
      discardLine = false;
    } else if (!discardLine) {
      if (byte == 0 || length == sizeof(line) - 1) {
        discardLine = true;
      } else {
        line[length++] = static_cast<char>(byte);
      }
    }
  }
}

