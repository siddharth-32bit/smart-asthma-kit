/*
 Smart Asthma Kit
 Firmware Module: Wristband
 Platform: ESP32-C6
*/

#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>

#define SDA_PIN 22
#define SCL_PIN 23

// ----------------------------
// BME680 Configuration
// ----------------------------

#define BME_ADDR 0x77

Adafruit_BME680 bme;

// ----------------------------
// MAX30102 Configuration
// ----------------------------

MAX30105 particleSensor;

// ----------------------------
// Sampling Configuration
// ----------------------------

#define SAMPLE_RATE_HZ 200
#define BUFFER_SIZE 100
#define SHIFT 25

uint32_t irBuffer[BUFFER_SIZE];
uint32_t redBuffer[BUFFER_SIZE];

int bufferIndex = 0;
int samplesCollected = 0;
int newSamples = 0;

// ----------------------------
// Timing
// ----------------------------

unsigned long lastPrintMillis = 0;
unsigned long lastBmeMillis = 0;

#define PRINT_INTERVAL_MS 1000
#define BME_INTERVAL_MS 2000

// ----------------------------
// SpO2 Smoothing
// ----------------------------

#define SPO2_SMOOTH_WINDOW 4

float spo2History[SPO2_SMOOTH_WINDOW];
int spo2HistoryIndex = 0;

// ----------------------------
// Latest Values
// ----------------------------

float lastHeartRate = 0.0;
bool lastHeartRateValid = false;

float lastSpO2 = 0.0;
bool lastSpO2Valid = false;

bool signalQualityGood = false;

// ----------------------------
// LED Configuration
// ----------------------------

const int LED_FIXED = 0x10;

const int LED_MIN_SAFE = 0x02;
const int LED_MAX_SAFE = 0x40;

// ----------------------------
// Signal Validation
// ----------------------------

#define IR_LOWER_OK 15000
#define IR_UPPER_OK 150000

#define RATIO_MIN 0.25f
#define RATIO_MAX 1.5f

// ----------------------------
// Environmental Values
// ----------------------------

float lastTemp = NAN;
float lastHumidity = NAN;
float lastPressure = NAN;
float lastGas = NAN;

// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(115200);

  delay(100);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  Serial.println();
  Serial.println("=== Smart Asthma Kit - Wristband Module ===");

  initializeBME680();
  initializeMAX30102();

  clearBuffers();

  Serial.println("System initialized successfully.");
}

// ============================================================
// LOOP
// ============================================================

void loop() {

  collectPulseSamples();

  processPulseData();

  updateEnvironmentalData();

  printSystemData();

  yield();
}

// ============================================================
// SENSOR INITIALIZATION
// ============================================================

void initializeBME680() {

  if (!bme.begin(BME_ADDR)) {

    Serial.println("BME680 not detected.");

    while (1) {
      delay(1000);
    }
  }

  bme.setTemperatureOversampling(BME680_OS_8X);

  bme.setHumidityOversampling(BME680_OS_2X);

  bme.setPressureOversampling(BME680_OS_4X);

  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);

  bme.setGasHeater(320, 150);
}

void initializeMAX30102() {

  if (!particleSensor.begin(Wire)) {

    Serial.println("MAX30102 not detected.");

    while (1) {
      delay(1000);
    }
  }

  uint8_t sampleAverage = 1;
  uint8_t ledMode = 2;

  uint8_t sampleRate = SAMPLE_RATE_HZ;

  uint16_t pulseWidth = 215;
  uint16_t adcRange = 4096;

  uint8_t ledPower = constrain(
    LED_FIXED,
    LED_MIN_SAFE,
    LED_MAX_SAFE
  );

  particleSensor.setup(
    ledPower,
    sampleAverage,
    ledMode,
    sampleRate,
    pulseWidth,
    adcRange
  );

  particleSensor.setPulseAmplitudeRed(ledPower);
  particleSensor.setPulseAmplitudeIR(ledPower);

  Serial.print("LED Power: ");
  Serial.println(ledPower);
}

// ============================================================
// BUFFER MANAGEMENT
// ============================================================

void clearBuffers() {

  for (int i = 0; i < BUFFER_SIZE; i++) {

    irBuffer[i] = 0;
    redBuffer[i] = 0;
  }

  for (int i = 0; i < SPO2_SMOOTH_WINDOW; i++) {

    spo2History[i] = 0.0;
  }
}

void copyBufferWindow(
  uint32_t *destinationIR,
  uint32_t *destinationRed
) {

  int start = bufferIndex - BUFFER_SIZE;

  if (start < 0) {
    start += BUFFER_SIZE;
  }

  for (int i = 0; i < BUFFER_SIZE; i++) {

    int index = start + i;

    if (index >= BUFFER_SIZE) {
      index -= BUFFER_SIZE;
    }

    destinationIR[i] = irBuffer[index];
    destinationRed[i] = redBuffer[index];
  }
}

uint32_t calculateAverage(
  uint32_t *buffer,
  int length
) {

  uint64_t sum = 0;

  for (int i = 0; i < length; i++) {
    sum += buffer[i];
  }

  return sum / length;
}

// ============================================================
// PULSE DATA COLLECTION
// ============================================================

void collectPulseSamples() {

  particleSensor.check();

  uint32_t red = particleSensor.getRed();
  uint32_t ir = particleSensor.getIR();

  irBuffer[bufferIndex] = ir;
  redBuffer[bufferIndex] = red;

  bufferIndex =
    (bufferIndex + 1) % BUFFER_SIZE;

  if (samplesCollected < BUFFER_SIZE) {
    samplesCollected++;
  }

  newSamples++;
}

// ============================================================
// PROCESS SPO2 & HEART RATE
// ============================================================

void processPulseData() {

  if (samplesCollected < BUFFER_SIZE ||
      newSamples < SHIFT) {
    return;
  }

  newSamples = 0;

  static uint32_t windowIR[BUFFER_SIZE];
  static uint32_t windowRed[BUFFER_SIZE];

  copyBufferWindow(windowIR, windowRed);

  uint32_t irAverage =
    calculateAverage(windowIR, BUFFER_SIZE);

  uint32_t redAverage =
    calculateAverage(windowRed, BUFFER_SIZE);

  float ratio =
    (irAverage > 0)
      ? (float)redAverage / irAverage
      : 0.0f;

  bool signalValid = true;

  if (irAverage < IR_LOWER_OK) {
    signalValid = false;
  }

  if (irAverage > IR_UPPER_OK) {
    signalValid = false;
  }

  if (ratio < RATIO_MIN ||
      ratio > RATIO_MAX) {
    signalValid = false;
  }

  if (!signalValid) {

    lastHeartRateValid = false;
    lastSpO2Valid = false;

    signalQualityGood = false;

    return;
  }

  int32_t spo2 = 0;
  int8_t spo2Valid = 0;

  int32_t heartRate = 0;
  int8_t heartRateValid = 0;

  maxim_heart_rate_and_oxygen_saturation(
    windowIR,
    BUFFER_SIZE,
    windowRed,
    &spo2,
    &spo2Valid,
    &heartRate,
    &heartRateValid
  );

  // Heart rate filtering
  if (heartRateValid == 1 &&
      heartRate > 30 &&
      heartRate < 220) {

    const float alpha = 0.35f;

    if (lastHeartRateValid) {

      lastHeartRate =
        alpha * heartRate +
        (1 - alpha) * lastHeartRate;
    }
    else {

      lastHeartRate = heartRate;
    }

    lastHeartRateValid = true;
  }
  else {

    lastHeartRateValid = false;
  }

  // SpO2 filtering
  if (spo2Valid == 1 &&
      spo2 >= 50 &&
      spo2 <= 100) {

    lastSpO2 = spo2;

    lastSpO2Valid = true;

    spo2History[spo2HistoryIndex] =
      lastSpO2;
  }
  else {

    lastSpO2Valid = false;

    spo2History[spo2HistoryIndex] = 0.0;
  }

  spo2HistoryIndex =
    (spo2HistoryIndex + 1) %
    SPO2_SMOOTH_WINDOW;

  signalQualityGood =
    lastHeartRateValid &&
    lastSpO2Valid;
}

// ============================================================
// ENVIRONMENTAL SENSOR
// ============================================================

void updateEnvironmentalData() {

  if (millis() - lastBmeMillis <
      BME_INTERVAL_MS) {
    return;
  }

  lastBmeMillis = millis();

  if (!bme.performReading()) {

    Serial.println("BME680 read failed.");

    return;
  }

  lastTemp = bme.temperature;

  lastHumidity = bme.humidity;

  lastPressure =
    bme.pressure / 100.0;

  lastGas =
    bme.gas_resistance;
}

// ============================================================
// SERIAL OUTPUT
// ============================================================

void printSystemData() {

  if (millis() - lastPrintMillis <
      PRINT_INTERVAL_MS) {
    return;
  }

  lastPrintMillis = millis();

  float spo2Average = 0.0;
  int validSamples = 0;

  for (int i = 0;
       i < SPO2_SMOOTH_WINDOW;
       i++) {

    if (spo2History[i] > 0.0) {

      spo2Average += spo2History[i];

      validSamples++;
    }
  }

  if (validSamples > 0) {

    spo2Average /= validSamples;
  }

  Serial.print("SpO2: ");

  if (validSamples > 0) {
    Serial.print(spo2Average, 1);
  }
  else {
    Serial.print("--");
  }

  Serial.print(" %");

  Serial.print(" | HR: ");

  if (lastHeartRateValid) {
    Serial.print(lastHeartRate, 1);
  }
  else {
    Serial.print("--");
  }

  Serial.print(" bpm");

  Serial.print(" | Signal: ");

  Serial.print(
    signalQualityGood
      ? "GOOD"
      : "BAD"
  );

  Serial.print(" | Temp: ");
  Serial.print(lastTemp, 1);

  Serial.print(" C");

  Serial.print(" | Humidity: ");
  Serial.print(lastHumidity, 1);

  Serial.print(" %");

  Serial.print(" | Pressure: ");
  Serial.print(lastPressure, 1);

  Serial.print(" hPa");

  Serial.print(" | Gas: ");
  Serial.print(lastGas, 0);

  Serial.println(" ohms");
}