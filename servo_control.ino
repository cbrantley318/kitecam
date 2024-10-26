#include <Arduino.h>
#include <ESP32Servo.h> // ESP32 compatible Servo library

// -------------------- Configuration Parameters --------------------

// -------------------- Servo Configuration --------------------
const int servoPin = 18;               // GPIO pin connected to the servo signal pin
Servo myServo;
int servoCurrentAngle = 90;            // Current servo angle (initially set to 90 degrees)
const int servoMin = 0;                 // Minimum servo angle
const int servoMax = 180;               // Maximum servo angle

// -------------------- Stepper Configuration --------------------
const int stepperPin1 = 19;            // IN1 on ULN2003
const int stepperPin2 = 21;            // IN2 on ULN2003
const int stepperPin3 = 22;            // IN3 on ULN2003
const int stepperPin4 = 23;            // IN4 on ULN2003

const int stepsPerRevolution = 2048;    // Total steps per full revolution for 28BYJ-48

// Define the step sequence for ULN2003 (4-wire, unipolar stepper motor)
const int stepSequence[4][4] = {
  {HIGH, LOW, LOW, LOW},   // Step 1
  {LOW, HIGH, LOW, LOW},   // Step 2
  {LOW, LOW, HIGH, LOW},   // Step 3
  {LOW, LOW, LOW, HIGH}    // Step 4
};

int currentStep = 0;
const int stepDelay = 5;               // Delay between steps in milliseconds

// -------------------- Joystick Configuration --------------------
const int joystickXPin = 34;           // GPIO34 (ADC1_CH6) - Joystick X-axis
const int joystickYPin = 35;           // GPIO35 (ADC1_CH7) - Joystick Y-axis

const int joystickDeadzone = 1500;      // Deadzone threshold
const int adcMax = 4095;               // Maximum ADC value for ESP32 (12-bit ADC)
const int adcMid = adcMax / 2;         // Middle value

// Timing for stepper motor control via joystick
unsigned long lastStepperStepTime = 0;
const unsigned long stepperInterval = 100; // Interval between stepper steps in milliseconds

// Timing for servo control via joystick
const int servoAdjustmentDelay = 100;  // Minimum delay between servo adjustments in milliseconds
unsigned long lastServoAdjustTime = 0;

// -------------------- Setup Function --------------------
void setup() {
  // Initialize Serial Communication
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect (only needed for native USB)
  }
  Serial.println("ESP32 Servo and Stepper Controller Initialized.");

  // Initialize Servo
  myServo.attach(servoPin, 500, 2400);  // Attach servo with specified min and max pulse widths
  myServo.write(servoCurrentAngle);      // Set initial servo position
  Serial.print("Servo initialized to ");
  Serial.print(servoCurrentAngle);
  Serial.println(" degrees.");

  // Initialize Stepper Pins
  pinMode(stepperPin1, OUTPUT);
  pinMode(stepperPin2, OUTPUT);
  pinMode(stepperPin3, OUTPUT);
  pinMode(stepperPin4, OUTPUT);

  // Initialize stepper to off (all LOW)
  stepperOff();
  Serial.println("Stepper motor initialized to OFF state.");
}

// -------------------- Main Loop --------------------
void loop() {
  handleSerialCommands();
  handleJoystickInputs();
}

// -------------------- Function Definitions --------------------

// Function to handle serial commands 'a' and 'b' for servo control
void handleSerialCommands() {
  while (Serial.available() > 0) {
    char command = Serial.read();
    command = tolower(command); // Convert to lowercase for consistency
    Serial.print("Received serial command: ");
    Serial.println(command);

    switch (command) {
      case 'a': // Decrease servo angle by 1 degree
        adjustServoAngle(-1);
        break;
      case 'b': // Increase servo angle by 1 degree
        adjustServoAngle(1);
        break;
      default:
        Serial.println("Unknown serial command.");
        break;
    }
  }
}

// Function to handle joystick inputs for servo and stepper control
void handleJoystickInputs() {
  // Read analog values from joystick
  int joyX = analogRead(joystickXPin);
  int joyY = analogRead(joystickYPin);

  // Debugging: Print joystick values
  // Serial.print("Joystick X: "); Serial.print(joyX);
  // Serial.print(" | Joystick Y: "); Serial.println(joyY);

  // Handle Joystick Y-axis (Up/Down) for Servo Control
  if (abs(joyY - adcMid) > joystickDeadzone) {
    unsigned long currentTime = millis();
    if (currentTime - lastServoAdjustTime >= servoAdjustmentDelay) {
      if (joyY > adcMid + joystickDeadzone) {
        // Joystick pushed up - Increase servo angle
        adjustServoAngle(5);
      } else if (joyY < adcMid - joystickDeadzone) {
        // Joystick pushed down - Decrease servo angle
        adjustServoAngle(-5);
      }
      lastServoAdjustTime = currentTime;
    }
  }

  // Handle Joystick X-axis (Left/Right) for Stepper Control
  if (abs(joyX - adcMid) > joystickDeadzone) {
    unsigned long currentTime = millis();
    if (currentTime - lastStepperStepTime >= stepperInterval) {
      if (joyX > adcMid + joystickDeadzone) {
        // Joystick pushed right - Rotate stepper clockwise
        rotateStepper(5); // Rotate one step clockwise
        Serial.println("Stepper motor rotated CLOCKWISE via joystick.");
      } else if (joyX < adcMid - joystickDeadzone) {
        // Joystick pushed left - Rotate stepper counter-clockwise
        rotateStepper(-5); // Rotate one step counter-clockwise
        Serial.println("Stepper motor rotated COUNTER-CLOCKWISE via joystick.");
      }
      lastStepperStepTime = currentTime;
    }
  }
}

// Function to adjust the servo angle with limits
void adjustServoAngle(int delta) {
  int newAngle = servoCurrentAngle + delta;

  // Ensure the new angle is within the defined limits
  if (newAngle < servoMin) {
    newAngle = servoMin;
    Serial.println("Servo angle reached MIN limit.");
  } else if (newAngle > servoMax) {
    newAngle = servoMax;
    Serial.println("Servo angle reached MAX limit.");
  }

  // Update servo position if angle has changed
  if (newAngle != servoCurrentAngle) {
    servoCurrentAngle = newAngle;
    myServo.write(servoCurrentAngle);
    Serial.print("Servo angle adjusted to: ");
    Serial.print(servoCurrentAngle);
    Serial.println(" degrees.");
  }
}

// Function to rotate the stepper motor by a specified number of steps
// Positive steps for clockwise, negative for counter-clockwise
void rotateStepper(int steps) {
  if (steps == 0) return;

  int stepDirection = steps > 0 ? 1 : -1;
  steps = abs(steps);

  for (int i = 0; i < steps; i++) {
    stepMotor(stepDirection);
    delay(stepDelay);
  }

  // Turn off all coils after movement to prevent overheating
  stepperOff();
}

// Function to execute one step in the specified direction
void stepMotor(int direction) {
  // Update the current step based on direction
  currentStep += direction;
  if (currentStep >= 4) {
    currentStep = 0;
  }
  if (currentStep < 0) {
    currentStep = 3;
  }

  // Apply the step sequence to the stepper motor
  digitalWrite(stepperPin1, stepSequence[currentStep][0]);
  digitalWrite(stepperPin2, stepSequence[currentStep][1]);
  digitalWrite(stepperPin3, stepSequence[currentStep][2]);
  digitalWrite(stepperPin4, stepSequence[currentStep][3]);
}

// Function to turn off all stepper coils
void stepperOff() {
  digitalWrite(stepperPin1, LOW);
  digitalWrite(stepperPin2, LOW);
  digitalWrite(stepperPin3, LOW);
  digitalWrite(stepperPin4, LOW);
}