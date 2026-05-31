/*
 * Smart Asthma Kit
 * Firmware Module: Wristband Wearable
 * Target Platform: ESP32-C6
 */

#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>

// --- Hardware Pin Definitions ---
#define SDA_PIN 22
#define SCL_PIN 23

// --- Sensor Configurations ---
#define BME_ADDR 0x77
Adafruit_BME680 bme;
MAX30105 particleSensor;

// --- Sampling Adjustments ---
#define SAMPLE_RATE_HZ 200
#define BUFFER_SIZE 100
#define SHIFT 25

// Cyclic Ring Buffers for PPG
uint32_t irBuffer[BUFFER_SIZE];
uint32_t redBuffer[BUFFER_SIZE];
int bufferIndex = 0;
int samplesCollected = 0;
int newSamples = 0;

// --- Asynchronous Execution Timing ---
uint32_t lastPrintMillis = 0;
uint32_t lastBmeMillis = 0;
const uint32_t PRINT_INTERVAL_MS = 1000;
const uint32_t BME_INTERVAL_MS = 2000; // Asynchronous polling interval

// --- Moving Window Metrics ---
#define SPO2_SMOOTH_WINDOW 4
float spo2History[SPO2_SMOOTH_WINDOW];
int spo2HistoryIndex = 0;

// Vitals State Machine
float lastHeartRate = 0.0f;
bool lastHeartRateValid = false;
float lastSpO2 = 0.0f;
bool lastSpO2Valid = false;
bool signalQualityGood = false;

// Signal Verification Thresholds
const int LED_POWER_LEVEL = 0x10; 
#define IR_LOWER_OK 15000
#define IR_UPPER_OK 150000
#define RATIO_MIN 0.25f
#define RATIO_MAX 1.5f

// Environmental State Machine
float lastTemp = NAN;
float lastHumidity = NAN;
float lastPressure = NAN;
float lastGas = NAN;

// --- Function Declarations ---
void initializeBME680();
void initializeMAX30102();
void clearBuffers();
void collectPulseSamples();
void processPulseData();
void updateEnvironmentalData();
void printSystemData();
void copyBufferWindow(uint32_t *destinationIR, uint32_t *destinationRed);
uint32_t calculateAverage(uint32_t *buffer, int length);

void setup() {
  Serial.begin(115200);
  delay(500); // Allow power supply to settle

  Serial.println(F("\n=== Smart Asthma Kit - Wristband Module ==="));

  // Initialize Hardware I2C on specified Pins
  if (!Wire.begin(SDA_PIN, SCL_PIN, 400000U)) {
    Serial.println(F("Critical Error: Failed to assign I2C Master Bus."));
    while (1) { delay(1000); }
  }

  initializeBME680();
  initializeMAX30102();
  clearBuffers();

  Serial.println(F("System initialized successfully.\n"));
}

void loop() {
  // Capture dynamic pulse changes at high priority
  collectPulseSamples();
  
  // Asynchronously isolate and analyze mathematical variations
  processPulseData();
  updateEnvironmentalData();
  printSystemData();
  
  yield(); // Keep the ESP32 background tasks running stably
}

void initializeBME680() {
  if (!bme.begin(BME_ADDR, &Wire)) {
    Serial.println(F("Critical Error: BME680 Environmental Sensor not found."));
    while (1) { delay(1000); }
  }

  // Adjust parameters for low-latency sampling
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320°C for 150ms
}

void initializeMAX30102() {
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println(F("Critical Error: MAX30102 SpO2 Sensor not found."));
    while (1) { delay(1000); }
  }

  uint8_t sampleAverage = 1;
  uint8_t ledMode = 2; // Red + IR mode
  uint16_t pulseWidth = 215;
  uint16_t adcRange = 4096;

  particleSensor.setup(
    LED_POWER_LEVEL,
    sampleAverage,
    ledMode,
    SAMPLE_RATE_HZ,
    pulseWidth,
    adcRange
  );

  particleSensor.setPulseAmplitudeRed(LED_POWER_LEVEL);
  particleSensor.setPulseAmplitudeIR(LED_POWER_LEVEL);
}

void clearBuffers() {
  memset(irBuffer, 0, sizeof(irBuffer));
  memset(redBuffer, 0, sizeof(redBuffer));
  for (int i = 0; i < SPO2_SMOOTH_WINDOW; i++) {
    spo2History[i] = 0.0f;
  }
}

void collectPulseSamples() {
  // Read sensor internal FIFO queue non-blockingly
  particleSensor.check();

  while (particleSensor.available()) {
    uint32_t red = particleSensor.getRed();
    uint32_t ir = particleSensor.getIR();

    irBuffer[bufferIndex] = ir;
    redBuffer[bufferIndex] = red;

    bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;

    if (samplesCollected < BUFFER_SIZE) {
      samplesCollected++;
    }
    newSamples++;
    
    particleSensor.nextSample(); // Clear current entry from hardware FIFO
  }
}

void processPulseData() {
  // Execute feature extraction only when window step criteria matches
  if (samplesCollected < BUFFER_SIZE || newSamples < SHIFT) {
    return;
  }

  newSamples = 0;

  uint32_t windowIR[BUFFER_SIZE];
  uint32_t windowRed[BUFFER_SIZE];
  copyBufferWindow(windowIR, windowRed);

  uint32_t irAverage = calculateAverage(windowIR, BUFFER_SIZE);
  uint32_t redAverage = calculateAverage(windowRed, BUFFER_SIZE);

  float ratio = (irAverage > 0) ? (float)redAverage / irAverage : 0.0f;
  bool signalValid = (irAverage >= IR_LOWER_OK && irAverage <= IR_UPPER_OK) && 
                     (ratio >= RATIO_MIN && ratio <= RATIO_MAX);

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

  // Run Maxim algorithmic DSP code block
  maxim_heart_rate_and_oxygen_saturation(
    windowIR, BUFFER_SIZE, windowRed,
    &spo2, &spo2Valid, &heartRate, &heartRateValid
  );

  // Digital Low Pass filtering for heart rate output
  if (heartRateValid == 1 && heartRate > 30 && heartRate < 220) {
    const float alpha = 0.35f;
    lastHeartRate = lastHeartRateValid ? (alpha * heartRate + (1.0f - alpha) * lastHeartRate) : (float)heartRate;
    lastHeartRateValid = true;
  } else {
    lastHeartRateValid = false;
  }

  // SpO2 data processing
  if (spo2Valid == 1 && spo2 >= 50 && spo2 <= 100) {
    lastSpO2 = (float)spo2;
    lastSpO2Valid = true;
    spo2History[spo2HistoryIndex] = lastSpO2;
  } else {
    lastSpO2Valid = false;
    spo2History[spo2HistoryIndex] = 0.0f;
  }

  spo2HistoryIndex = (spo2HistoryIndex + 1) % SPO2_SMOOTH_WINDOW;
  signalQualityGood = lastHeartRateValid && lastSpO2Valid;
}

void updateEnvironmentalData() {
  uint32_t currentMillis = millis();
  if (currentMillis - lastBmeMillis < BME_INTERVAL_MS) {
    return;
  }
  lastBmeMillis = currentMillis;

  // Read atmospheric data asynchronously
  if (!bme.performReading()) {
    Serial.println(F("Warning: Environmental reading skipped (Sensor Busy)."));
    return;
  }

  lastTemp = bme.temperature;
  lastHumidity = bme.humidity;
  lastPressure = bme.pressure / 100.0f; // Convert Pascal to hPa
  lastGas = bme.gas_resistance;
}

void printSystemData() {
  uint32_t currentMillis = millis();
  if (currentMillis - lastPrintMillis < PRINT_INTERVAL_MS) {
    return;
  }
  lastPrintMillis = currentMillis;

  float spo2Average = 0.0f;
  int validSamples = 0;

  for (int i = 0; i < SPO2_SMOOTH_WINDOW; i++) {
    if (spo2History[i] > 0.0f) {
      spo2Average += spo2History[i];
      validSamples++;
    }
  }
  if (validSamples > 0) {
    spo2Average /= validSamples;
  }

  Serial.print(F("SpO2: "));
  if (validSamples > 0) { Serial.print(spo2Average, 1); Serial.print(F(" %")); } 
  else { Serial.print(F("-- %")); }

  Serial.print(F(" | HR: "));
  if (lastHeartRateValid) { Serial.print(lastHeartRate, 1); Serial.print(F(" bpm")); } 
  else { Serial.print(F("-- bpm")); }

  Serial.print(F(" | Signal: ")); Serial.print(signalQualityGood ? F("GOOD") : F("BAD"));
  Serial.print(F(" | Temp: ")); Serial.print(lastTemp, 1); Serial.print(F(" C"));
  Serial.print(F(" | Humidity: ")); Serial.print(lastHumidity, 1); Serial.print(F(" %"));
  Serial.print(F(" | Pressure: ")); Serial.print(lastPressure, 1); Serial.print(F(" hPa"));
  Serial.print(F(" | Gas: ")); Serial.print(lastGas, 0); Serial.println(F(" ohms"));
}

void copyBufferWindow(uint32_t *destinationIR, uint32_t *destinationRed) {
  int start = bufferIndex - BUFFER_SIZE;
  if (start < 0) start += BUFFER_SIZE;

  for (int i = 0; i < BUFFER_SIZE; i++) {
    int index = start + i;
    if (index >= BUFFER_SIZE) index -= BUFFER_SIZE;

    destinationIR[i] = irBuffer[index];
    destinationRed[i] = redBuffer[index];
  }
}

uint32_t calculateAverage(uint32_t *buffer, int length) {
  uint64_t sum = 0;
  for (int i = 0; i < length; i++) {
    sum += buffer[i];
  }
  return sum / length;
}
