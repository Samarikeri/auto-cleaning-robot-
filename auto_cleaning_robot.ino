#include <Servo.h>

// === Motor driver pins ===
#define ENA 3
#define ENB 5
#define IN1 4
#define IN2 7
#define IN3 10
#define IN4 11

// === Ultrasonic sensor pins ===
#define TRIG_PIN 9
#define ECHO_PIN 8

// === Other sensor pins ===
#define IR_SENSOR A0
#define GAS_SENSOR A1

// === Loader servo ===
#define LOADER_SERVO_PIN 6
Servo loader;

// === Mode selection ===
bool isManual = false;  // Default to auto mode

void setup() {
  // Initialize serial for Bluetooth module
  Serial.begin(9600);

  // Motor pins
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // Sensor pins
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);
  pinMode(IR_SENSOR, INPUT);
  pinMode(GAS_SENSOR, INPUT);

  // Attach loader servo
  loader.attach(LOADER_SERVO_PIN);
  loader.write(0);  // Start with bucket down

  Serial.println("Robot Ready...");
}

void loop() {
  // Bluetooth input (Manual Mode)
  if (Serial.available()) {
    char input = Serial.read();
    handleBluetooth(input);
  }

  // Auto Mode logic
  if (!isManual) {
    autoMode();
  }
}

// === Bluetooth Commands ===
void handleBluetooth(char cmd) {
  isManual = true;
  switch (cmd) {
    case 'F': moveForward(); break;
    case 'B': moveBackward(); break;
    case 'L': turnLeft(); break;
    case 'R': turnRight(); break;
    case 'S': stopMotors(); break;
    case 'U': liftBucket(); break;
    case 'D': dropBucket(); break;
    case 'A': isManual = false; stopMotors(); break;
    default: stopMotors(); break;
  }
}

// === Autonomous Behavior ===
void autoMode() {
  float dist = readDistance();
  int irVal = digitalRead(IR_SENSOR);
  int gasVal = analogRead(GAS_SENSOR);

  // IR edge detection (floor drop)
  if (irVal == LOW) {
    stopMotors();
    moveBackward();
    delay(400);
    turnRight();
    delay(500);
    return;
  }

  // Object detected by ultrasonic
  if (dist < 20) {
    stopMotors();
    moveBackward();
    delay(400);
    turnLeft();
    delay(500);
    return;
  }

  // Check for garbage using gas sensor
  if (gasVal > 400) {
    stopMotors();
    liftBucket();
    delay(2000); // simulate bucket operation
    dropBucket();
    moveForward();
    delay(1000);
  } else {
    moveForward();
  }
}

// === Motor control ===
void moveForward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, 180); analogWrite(ENB, 180);
}

void moveBackward() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(ENA, 180); analogWrite(ENB, 180);
}

void turnLeft() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, 150); analogWrite(ENB, 150);
}

void turnRight() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(ENA, 150); analogWrite(ENB, 150);
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

// === Loader Functions ===
void liftBucket() {
  loader.write(90); // Raise
}

void dropBucket() {
  loader.write(0); // Lower
}

// === Ultrasonic Distance ===
float readDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  float duration = pulseIn(ECHO_PIN, HIGH);
  return duration * 0.034 / 2;  // Convert to cm
}
