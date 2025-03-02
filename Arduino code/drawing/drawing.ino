#include <AccelStepper.h>
#include <ESP32Servo.h>

#define SERVO_PIN 18  // GPIO pin for servo

// Define min/max angles for servo
#define PEN_UP_ANGLE 90   // Adjust for your setup
#define PEN_DOWN_ANGLE 0  // Adjust for your setup

Servo penServo;  // Create a Servo object

// Define lengths of robot arms (in mm)
const float L1 = 100.0;
const float L2 = 100.0;

// Initialize stepper motors (step, direction pins)
AccelStepper stepper1(AccelStepper::DRIVER, 2, 5);
AccelStepper stepper2(AccelStepper::DRIVER, 3, 6);

void setup() {
  Serial.begin(115200);

  // Setup stepper motors
  stepper1.setMaxSpeed(1000);
  stepper1.setAcceleration(500);
  stepper2.setMaxSpeed(1000);
  stepper2.setAcceleration(500);

  // Attach servo to the defined pin
  penServo.attach(SERVO_PIN);

  penUp(); // Start with pen up
}

void loop() {
  moveTo(50, 50);
  delay(2000);
}

// Move pen to (x, y) using inverse kinematics
void moveTo(float x, float y) {
  float theta1, theta2;
  if (inverseKinematics(x, y, theta1, theta2)) {
    long steps1 = angleToSteps(theta1);
    long steps2 = angleToSteps(theta2);

    stepper1.moveTo(steps1);
    stepper2.moveTo(steps2);

    while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0) {
      stepper1.run();
      stepper2.run();
    }
  } else {
    Serial.println("Position out of reach");
  }
}

// Inverse kinematics function
bool inverseKinematics(float x, float y, float &theta1, float &theta2) {
  float distance = sqrt(x * x + y * y);
  if (distance > (L1 + L2) || distance < abs(L1 - L2)) {
    return false;
  }
  float cosTheta2 = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2);
  theta2 = acos(cosTheta2);
  float k1 = L1 + L2 * cos(theta2);
  float k2 = L2 * sin(theta2);
  theta1 = atan2(y, x) - atan2(k2, k1);
  return true;
}

// Convert angle to stepper motor steps
long angleToSteps(float angle) {
  const float stepsPerRevolution = 200;
  const float radiansPerStep = 2 * PI / stepsPerRevolution;
  return long(angle / radiansPerStep);
}

// Move pen up
void penUp() {
  penServo.write(PEN_UP_ANGLE);
  delay(500);
}

// Move pen down
void penDown() {
  penServo.write(PEN_DOWN_ANGLE);
  delay(500);
}
