/*
 * Smart Asthma Kit
 * Firmware Module: Spirometer
 * Target Platform: Arduino UNO (ATmega328P)
 */

#include <Arduino.h>

// --- Configuration Constants ---
const uint8_t ANALOG_PIN = A0;
const float VS = 5.00f;              // Supply voltage (Volts)
float K = 35.00f;                    // Calibration constant

// Sampling configuration
const uint16_t SAMPLE_HZ = 100;
const uint32_t SAMPLE_US = 1000000UL / SAMPLE_HZ;
const uint32_t MAX_MEAS_MS = 6000UL;

// End-of-test detection
const float CMH2O_TO_ZERO = 0.3f;
const uint8_t LOW_COUNT_TO_END = SAMPLE_HZ / 4; // 250ms of near-zero flow

// Serial communication buffer
const uint8_t SERIAL_BUF_SIZE = 32;

// --- User Demographics ---
int user_age = 30;
char user_sex = 'M';
int user_height_cm = 170;

// --- Function Declarations ---
void promptDemographics();
void waitForNumber(int *outVal);
void waitForSex();
void handleSerialCommands();
void runTest();
float predictedFEV1(int age, char sex, int height_cm);

void setup() {
  Serial.begin(115200);
  analogReference(DEFAULT);
  delay(200);

  Serial.println(F("\n=== Smart Asthma Kit - Spirometry Test ==="));
  promptDemographics();
  Serial.println(F("\nPress ENTER to start a new test or type 'K <val>' to calibrate."));
}

void loop() {
  if (Serial.available() > 0) {
    handleSerialCommands();
  }
}

void handleSerialCommands() {
  static char buffer[SERIAL_BUF_SIZE];
  static uint8_t index = 0;

  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (index == 0) {
        // Empty line received -> Trigger Test execution
        runTest();
        return;
      }
      
      buffer[index] = '\0'; // Null-terminate string
      
      // Parse calibration constant
      if ((buffer[0] == 'k' || buffer[0] == 'K') && buffer[1] == ' ') {
        float value = atof(&buffer[2]);
        if (value > 0.0f) {
          K = value;
          Serial.print(F("Calibration constant updated to: "));
          Serial.println(K, 3);
        } else {
          Serial.println(F("Usage error. Format: K <value> (e.g., K 35.5)"));
        }
      }
      
      index = 0; // Reset buffer pointer
    } else if (index < SERIAL_BUF_SIZE - 1) {
      buffer[index++] = c;
    }
  }
}

void promptDemographics() {
  Serial.print(F("Enter age (years): "));
  waitForNumber(&user_age);

  Serial.print(F("Enter sex (M/F): "));
  waitForSex();

  Serial.print(F("Enter height (cm): "));
  waitForNumber(&user_height_cm);

  Serial.print(F("\nConfigured Patient Profile -> Age: "));
  Serial.print(user_age);
  Serial.print(F(" | Sex: "));
  Serial.print(user_sex);
  Serial.print(F(" | Height: "));
  Serial.print(user_height_cm);
  Serial.println(F(" cm"));
}

void waitForNumber(int *outVal) {
  while (true) {
    if (Serial.available() > 0) {
      String input = Serial.readStringUntil('\n');
      input.trim();
      int value = input.toInt();
      if (value > 0) {
        *outVal = value;
        break;
      }
      Serial.println(F("Invalid dynamic payload. Please enter a valid positive integer:"));
    }
  }
}

void waitForSex() {
  while (true) {
    if (Serial.available() > 0) {
      String input = Serial.readStringUntil('\n');
      input.trim();
      if (input.length() > 0) {
        char value = toupper(input.charAt(0));
        if (value == 'M' || value == 'F') {
          user_sex = value;
          break;
        }
      }
      Serial.println(F("Invalid entry. Please type M or F:"));
    }
  }
}

void runTest() {
  Serial.println(F("\n--- Preparing Spirometer Sensor ---"));
  Serial.println(F("1) Inhale completely (Deep Breath)"));
  Serial.println(F("2) Form a tight seal around the mouthpiece"));
  Serial.println(F("3) Exhale as forcefully and quickly as possible!"));

  for (int i = 3; i >= 1; i--) {
    Serial.print(i);
    Serial.println(F("..."));
    delay(1000);
  }
  Serial.println(F("BLOW NOW!"));

  uint32_t startMs = millis();
  uint32_t deadline = startMs + MAX_MEAS_MS;
  uint32_t lastSampleMicros = micros();

  float peakFlowLmin = 0.0f;
  double totalVolumeL = 0.0;
  double fev1_L = 0.0;
  float prevFlow_Ls = 0.0f;
  uint8_t lowCount = 0;

  while (millis() < deadline) {
    uint32_t currentMicros = micros();

    // Safe rollover-resistant timing check
    if (currentMicros - lastSampleMicros >= SAMPLE_US) {
      lastSampleMicros += SAMPLE_US;
      uint32_t nowMs = millis();

      // Read pressure data
      int adc = analogRead(ANALOG_PIN);
      float voltage = adc * (VS / 1023.0f);
      float pressure_kPa = (voltage / VS - 0.04f) / 0.09f;
      
      if (pressure_kPa < 0.0f) pressure_kPa = 0.0f;
      float pressure_cmh2o = pressure_kPa * 10.197f;

      // Mathematical flow translation
      float flowLmin = K * sqrtf(pressure_cmh2o);
      if (!isfinite(flowLmin) || flowLmin < 0.0f) flowLmin = 0.0f;

      if (flowLmin > peakFlowLmin) {
        peakFlowLmin = flowLmin;
      }

      // Convert from L/min to L/s
      float currentFlow_Ls = flowLmin / 60.0f;
      double dt = (double)SAMPLE_US / 1000000.0;

      // High-precision Trapezoidal Integration for volumes
      double incrementalVolume = 0.5 * (currentFlow_Ls + prevFlow_Ls) * dt;
      totalVolumeL += incrementalVolume;

      if (nowMs - startMs <= 1000UL) {
        fev1_L += incrementalVolume;
      }
      
      prevFlow_Ls = currentFlow_Ls;

      // End-of-exhalation auto detection
      if (pressure_cmh2o <= CMH2O_TO_ZERO) {
        lowCount++;
      } else {
        lowCount = 0;
      }

      if (lowCount >= LOW_COUNT_TO_END && (nowMs - startMs) > 500) {
        break; // Stop acquisition early if patient finished blowing
      }
    }
  }

  uint32_t durationMs = millis() - startMs;
  float PEF = peakFlowLmin;
  float FEV1 = (float)fev1_L;
  float FVC = (float)totalVolumeL;

  if (FVC < FEV1) FVC = FEV1;

  float predicted = predictedFEV1(user_age, user_sex, user_height_cm);
  float ratio = (predicted > 0.0f) ? (FEV1 / predicted) : 0.0f;
  
  const char *health = (ratio >= 0.80f) ? "Good" : ((ratio >= 0.60f) ? "Average" : "Bad");

  // Output test results
  Serial.println(F("\n================ TEST RESULTS ================"));
  Serial.print(F("Exhalation Duration : ")); Serial.print(durationMs); Serial.println(F(" ms"));
  Serial.print(F("PEF (Peak Flow)     : ")); Serial.print(PEF, 1); Serial.println(F(" L/min"));
  Serial.print(F("FEV1                : ")); Serial.print(FEV1, 3); Serial.println(F(" L"));
  Serial.print(F("FVC                 : ")); Serial.print(FVC, 3); Serial.println(F(" L"));
  Serial.print(F("Predicted FEV1      : ")); Serial.print(predicted, 3); Serial.println(F(" L"));
  Serial.print(F("FEV1 Performance    : ")); Serial.print(ratio * 100.0f, 0); Serial.println(F(" %"));
  Serial.print(F("Clinical Assessment : ")); Serial.println(health);
  Serial.println(F("=============================================="));
}

float predictedFEV1(int age, char sex, int height_cm) {
  float h = (float)height_cm;
  float a = (float)age;

  if (sex == 'M') {
    return max(0.5f, 0.041f * h - 0.024f * a - 2.69f);
  } else {
    return max(0.5f, 0.034f * h - 0.025f * a - 1.90f);
  }
}
