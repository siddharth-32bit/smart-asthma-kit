/*
 Smart Asthma Kit
 Firmware Module: Spirometer
 Platform: Arduino UNO
*/

const int ANALOG_PIN = A0;
const float VS = 5.00f;

// Calibration constant
float K = 35.00f;

// Sampling configuration
const int SAMPLE_HZ = 100;
const unsigned long SAMPLE_US = 1000000UL / SAMPLE_HZ;
const unsigned long MAX_MEAS_MS = 6000UL;

// End detection
const float CMH2O_TO_ZERO = 0.3f;
const int LOW_COUNT_TO_END = SAMPLE_HZ / 4;

// User demographics
int user_age = 30;
char user_sex = 'M';
int user_height_cm = 170;

void setup() {
  Serial.begin(115200);
  analogReference(DEFAULT);

  delay(200);

  Serial.println();
  Serial.println("=== Smart Asthma Kit - Spirometry Test ===");

  promptDemographics();

  Serial.println();
  Serial.println("Press ENTER to start the test.");
}

void loop() {

  // Start test on ENTER
  if (Serial.available()) {

    String line = Serial.readStringUntil('\n');
    line.trim();

    // Calibration command
    if (line.startsWith("k ") || line.startsWith("K ")) {

      float value = line.substring(2).toFloat();

      if (value > 0.0f) {
        K = value;

        Serial.print("Calibration constant updated: ");
        Serial.println(K, 3);
      }
      else {
        Serial.println("Usage: k <value>");
      }

      return;
    }

    // Empty line starts test
    if (line.length() == 0) {
      runTest();
    }
  }
}

void promptDemographics() {

  Serial.print("Enter age (years): ");
  waitForNumber(&user_age);

  Serial.print("Enter sex (M/F): ");
  waitForSex();

  Serial.print("Enter height (cm): ");
  waitForNumber(&user_height_cm);

  Serial.println();

  Serial.print("Using Age=");
  Serial.print(user_age);

  Serial.print("  Sex=");
  Serial.print(user_sex);

  Serial.print("  Height=");
  Serial.print(user_height_cm);

  Serial.println(" cm");
}

void waitForNumber(int *outVal) {

  while (true) {

    while (!Serial.available()) {
      delay(20);
    }

    String input = Serial.readStringUntil('\n');
    input.trim();

    int value = input.toInt();

    if (value > 0) {
      *outVal = value;
      break;
    }

    Serial.println("Please enter a valid number:");
  }
}

void waitForSex() {

  while (true) {

    while (!Serial.available()) {
      delay(20);
    }

    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() > 0) {

      char value = toupper(input.charAt(0));

      if (value == 'M' || value == 'F') {
        user_sex = value;
        break;
      }
    }

    Serial.println("Please enter M or F:");
  }
}

void runTest() {

  Serial.println();
  Serial.println("Get ready...");
  Serial.println("1) Take a deep breath");
  Serial.println("2) Seal lips on mouthpiece");
  Serial.println("3) Blow as hard and fast as possible");

  for (int i = 3; i >= 1; i--) {

    Serial.print(i);
    Serial.println("...");

    delay(700);
  }

  Serial.println("Start now!");

  unsigned long startMs = millis();
  unsigned long deadline = startMs + MAX_MEAS_MS;
  unsigned long nextMicros = micros();

  float peakFlowLmin = 0.0f;
  double totalVolumeL = 0.0;
  double fev1_L = 0.0;

  int lowCount = 0;

  while (millis() < deadline) {

    unsigned long currentMicros = micros();

    if (currentMicros < nextMicros) {
      continue;
    }

    nextMicros += SAMPLE_US;

    unsigned long nowMs = millis();

    // Sensor reading
    int adc = analogRead(ANALOG_PIN);

    float voltage = adc * (VS / 1023.0f);

    float pressure_kPa =
      (voltage / VS - 0.04f) / 0.09f;

    if (pressure_kPa < 0.0f) {
      pressure_kPa = 0.0f;
    }

    float pressure_cmh2o =
      pressure_kPa * 10.197f;

    // Flow calculation
    float flowLmin =
      K * sqrtf(pressure_cmh2o);

    if (!isfinite(flowLmin) || flowLmin < 0.0f) {
      flowLmin = 0.0f;
    }

    // Peak flow
    if (flowLmin > peakFlowLmin) {
      peakFlowLmin = flowLmin;
    }

    // Volume integration
    float flow_Ls = flowLmin / 60.0f;

    totalVolumeL +=
      (double)flow_Ls / SAMPLE_HZ;

    // FEV1 calculation
    if (nowMs - startMs < 1000UL) {

      fev1_L +=
        (double)flow_Ls / SAMPLE_HZ;
    }

    // End detection
    if (pressure_cmh2o <= CMH2O_TO_ZERO) {
      lowCount++;
    }
    else {
      lowCount = 0;
    }

    if (lowCount >= LOW_COUNT_TO_END &&
        (nowMs - startMs) > 300) {
      break;
    }
  }

  // Final calculations
  unsigned long durationMs =
    millis() - startMs;

  float PEF = peakFlowLmin;
  float FEV1 = (float)fev1_L;
  float FVC = (float)totalVolumeL;

  if (FVC < FEV1) {
    FVC = FEV1;
  }

  float predicted =
    predictedFEV1(
      user_age,
      user_sex,
      user_height_cm
    );

  float ratio =
    (predicted > 0.0f)
      ? (FEV1 / predicted)
      : 0.0f;

  const char *health;

  if (ratio >= 0.80f) {
    health = "Good";
  }
  else if (ratio >= 0.60f) {
    health = "Average";
  }
  else {
    health = "Bad";
  }

  // Results
  Serial.println();
  Serial.println("----- TEST RESULT -----");

  Serial.print("Duration: ");
  Serial.print(durationMs);
  Serial.println(" ms");

  Serial.print("PEF: ");
  Serial.print(PEF, 1);
  Serial.println(" L/min");

  Serial.print("FEV1: ");
  Serial.print(FEV1, 3);
  Serial.println(" L");

  Serial.print("FVC: ");
  Serial.print(FVC, 3);
  Serial.println(" L");

  Serial.print("Predicted FEV1: ");
  Serial.print(predicted, 3);
  Serial.println(" L");

  Serial.print("FEV1 Ratio: ");
  Serial.print(ratio * 100.0f, 0);
  Serial.println(" %");

  Serial.print("Health Status: ");
  Serial.println(health);

  Serial.println("-----------------------");
}

float predictedFEV1(
  int age,
  char sex,
  int height_cm
) {

  float h = (float)height_cm;
  float a = (float)age;

  if (sex == 'M') {

    return max(
      0.5f,
      0.041f * h -
      0.024f * a -
      2.69f
    );
  }
  else {

    return max(
      0.5f,
      0.034f * h -
      0.025f * a -
      1.90f
    );
  }
}