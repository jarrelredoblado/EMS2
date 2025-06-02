#include <Wire.h>
#include <LiquidCrystal_I2C.h>   // Make sure you have this library installed

// LCD setup (address 0x27, 16 columns x 2 rows)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ADXL335 analog output pins
const int xPin = A0;
const int yPin = A1;
const int zPin = A2;
const int ST_PIN = 4;           // Self-test control pin (connected to gate of P-MOSFET)
const int buttonPin = 2;        // Calibration button pin

// Calibration variables
const char* directions[6] = {"+X", "-X", "+Y", "-Y", "+Z", "-Z"};
float calibrationData[6][3];  // Stores raw readings per calibration step
int calibStep = 0;

// Calibration state machine
enum State { SELF_TEST, /*WAIT_FOR_CALIB_START, CALIBRATION,*/ STEPPING };
State currentState = SELF_TEST;

// Button press tracking variables
bool buttonState = true;
bool lastPhysicalState = LOW;
unsigned long pressTime = 0;
const unsigned long pressDurationMicros = 800;  // 0.8 ms

// Constants for ADXL335 calibration (can be updated after calibration)
float ZERO_G_VOLTAGE_X = 2.08;
float ZERO_G_VOLTAGE_Y = 2.47;
float ZERO_G_VOLTAGE_Z = 3.00;

const float SENSITIVITY = 0.330;
const float ADC_REF = 5.0;
const int ADC_MAX = 1023;

// Step detection thresholds and parameters
const float STEP_PEAK_THRESHOLD = 2.0;
const float STEP_VALLEY_THRESHOLD = 1.5;
const int MIN_STEP_INTERVAL = 300;

// Moving average smoothing variables for magnitude
const int WINDOW_SIZE = 10;
float magnitudeHistory[WINDOW_SIZE];
int historyIndex = 0;
float magnitudeSum = 0.0;

// Step detection variables
unsigned long lastStepTime = 0;
int stepCount = 0;
bool stepDetected = false;
unsigned long stepDetectedTime = 0;
const unsigned long STEP_DETECT_TIMEOUT = 1000;

// Calibration prompt printed flag
bool promptPrinted = false;

// Function prototypes
void updateButtonState();
float readAxis(int pin, float zero_g_voltage);
float readAccel(int pin, float zero_g_voltage);
float readAccelMagnitude();
float getSmoothedMagnitude(float newMagnitude);
float readVoltage_mV(int pin);

void runSelfTest();
//void runWaitForCalibStart();
//void runCalibration();
void runStepDetection();

void setup() {
  Serial.begin(9600);

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();

  pinMode(ST_PIN, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  digitalWrite(ST_PIN, HIGH);

  Serial.println("Starting Self-Test...");
  lcd.setCursor(0, 0);
  lcd.print("Starting");
  lcd.setCursor(0, 1);
  lcd.print("Self-Test...");

  delay(2000);

  runSelfTest();
}

void loop() {
  updateButtonState();

  switch (currentState) {
    case SELF_TEST:
      // runSelfTest() blocks until finished and sets currentState
      break;

    /*case WAIT_FOR_CALIB_START:
      runWaitForCalibStart();
      break;

    case CALIBRATION:
      runCalibration();
      break;*/

    case STEPPING:
      runStepDetection();
      break;
  }
}

// Button reading and logical state management
void updateButtonState() {
  bool physicalState = digitalRead(buttonPin);  // LOW when pressed

  // Detect the press event (transition from HIGH to LOW)
  if (lastPhysicalState == HIGH && physicalState == LOW) {
    buttonState = true;
    pressTime = micros();
    Serial.println("Button pressed");
  }

  lastPhysicalState = physicalState;

  // Reset buttonState after 0.8 ms (800 microseconds)
  if (buttonState && (micros() - pressTime >= pressDurationMicros)) {
    buttonState = false;
  }
}

/*
void runWaitForCalibStart() {
  // Disabled
}

void runCalibration() {
  // Disabled
}
*/

void runStepDetection() {
  float magnitude = readAccelMagnitude();
  float smoothedMagnitude = getSmoothedMagnitude(magnitude);

  unsigned long currentTime = millis();

  // Detect steps on peaks above threshold and respect minimum interval
  if (!stepDetected && smoothedMagnitude > STEP_PEAK_THRESHOLD && (currentTime - lastStepTime) > MIN_STEP_INTERVAL) {
    stepCount++;
    lastStepTime = currentTime;
    stepDetected = true;
    stepDetectedTime = currentTime;

    Serial.print("Step detected! Total steps: ");
    Serial.println(stepCount);
  }

  // Reset stepDetected when smoothed magnitude falls below valley threshold
  if (stepDetected && smoothedMagnitude < STEP_VALLEY_THRESHOLD) {
    stepDetected = false;
    Serial.println("Step detection reset by valley");
  }

  // Failsafe reset if stuck too long
  if (stepDetected && (currentTime - stepDetectedTime > STEP_DETECT_TIMEOUT)) {
    stepDetected = false;
    Serial.println("Step detection reset by timeout");
  }

  // Serial output
  Serial.print("Magnitude: ");
  Serial.print(magnitude, 3);
  Serial.print(" | Smoothed: ");
  Serial.print(smoothedMagnitude, 3);
  Serial.print(" | Steps: ");
  Serial.println(stepCount);

  // LCD output (streamlined)
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Steps:");
  lcd.print(stepCount);
  lcd.setCursor(0, 1);
  lcd.print("Sm:");
  lcd.print(smoothedMagnitude, 2);

  delay(100);
}

void runSelfTest() {
  float x_initial_mV, y_initial_mV, z_initial_mV;
  float x_test_mV, y_test_mV, z_test_mV;
  float deltaX_mV, deltaY_mV, deltaZ_mV;

  delay(1000);

  // Read baseline voltage in mV
  x_initial_mV = readVoltage_mV(xPin);
  y_initial_mV = readVoltage_mV(yPin);
  z_initial_mV = readVoltage_mV(zPin);

  Serial.print("Initial Voltage (mV): X=");
  Serial.print(x_initial_mV);
  Serial.print(" Y=");
  Serial.print(y_initial_mV);
  Serial.print(" Z=");
  Serial.println(z_initial_mV);

  // LCD output
  lcd.clear();

  // Activate self-test (P-MOSFET: gate LOW → ST pin HIGH)
  digitalWrite(ST_PIN, LOW);
  delay(500); // stabilize

  // Read self-test voltage in mV
  x_test_mV = readVoltage_mV(xPin);
  y_test_mV = readVoltage_mV(yPin);
  z_test_mV = readVoltage_mV(zPin);

  // Deactivate self-test
  digitalWrite(ST_PIN, HIGH);

  Serial.print("Self-Test Voltage (mV): X=");
  Serial.print(x_test_mV);
  Serial.print(" Y=");
  Serial.print(y_test_mV);
  Serial.print(" Z=");
  Serial.println(z_test_mV);

  // LCD update

  // Compute delta voltage in mV
  deltaX_mV = x_test_mV - x_initial_mV;
  deltaY_mV = y_test_mV - y_initial_mV;
  deltaZ_mV = z_test_mV - z_initial_mV;

  // Your adjusted expected delta voltages and tolerances:
  const float expectedDeltaX = -325.0;
  const float expectedDeltaY = 325.0;
  const float expectedDeltaZ = 550.0;

  const float toleranceX = 225.0;
  const float toleranceY = 225.0;
  const float toleranceZ = 425.0;

  bool pass =
    (abs(deltaX_mV - expectedDeltaX) < toleranceX) &&
    (abs(deltaY_mV - expectedDeltaY) < toleranceY) &&
    (abs(deltaZ_mV - expectedDeltaZ) < toleranceZ);

  if (pass) {
    Serial.println("Self-Test Passed");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Self-Test");
    lcd.setCursor(0, 1);
    lcd.print("Passed");
    delay(3000);  // Show message for 3 seconds
    currentState = STEPPING;  // Directly go to step detection
  } else {
    Serial.println("Self-Test Failed");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Self-Test");
    lcd.setCursor(0, 1);
    lcd.print("Failed");
    while (true) delay(1000);
  }
}

float readVoltage_mV(int pin) {
  int raw = analogRead(pin);
  float voltage = (raw * ADC_REF) / ADC_MAX;
  return voltage * 1000.0;
}

float readAxis(int pin, float zero_g_voltage) {
  int raw = analogRead(pin);
  float voltage = (raw * ADC_REF) / ADC_MAX;
  return (voltage - zero_g_voltage) / SENSITIVITY;
}

float readAccel(int pin, float zero_g_voltage) {
  int raw = analogRead(pin);
  float voltage = (raw * ADC_REF) / ADC_MAX;
  float accel = (voltage - zero_g_voltage) / SENSITIVITY;
  return accel;
}

float readAccelMagnitude() {
  float ax = readAccel(xPin, ZERO_G_VOLTAGE_X);
  float ay = readAccel(yPin, ZERO_G_VOLTAGE_Y);
  float az = readAccel(zPin, ZERO_G_VOLTAGE_Z);
  return sqrt(ax * ax + ay * ay + az * az);
}

float getSmoothedMagnitude(float newMagnitude) {
  magnitudeSum -= magnitudeHistory[historyIndex];
  magnitudeHistory[historyIndex] = newMagnitude;
  magnitudeSum += newMagnitude;
  historyIndex = (historyIndex + 1) % WINDOW_SIZE;
  return magnitudeSum / WINDOW_SIZE;
}






