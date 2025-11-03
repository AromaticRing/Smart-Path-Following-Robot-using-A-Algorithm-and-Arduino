#include <NewPing.h>
#include <Servo.h>

// === Ultrasonic ===
#define TRIG_PIN A5
#define ECHO_PIN A4
#define MAX_DISTANCE 200
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);

// === Servo ===
#define SERVO_PIN 3
Servo servo;

// === Motor Driver Pins ===
#define IN1 5
#define IN2 6
#define IN3 7
#define IN4 8
#define ENA 9
#define ENB 11

// === Timing Calibration ===
const int ROTATE_DELAY_PER_DEG = 10;   // adjust for your motor speed
const int MOVE_DELAY_PER_CM    = 15;   // adjust for your car’s speed

void setup() {
  Serial.begin(9600);
  servo.attach(SERVO_PIN);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);

  stopMotors();
  Serial.println("READY");
  delay(500);
}

void loop() {
  // === Mapping Mode ===
  if (Serial.available() == 0) {
    for (int ang = 0; ang <= 180; ang += 10) {
      servo.write(ang);
      delay(150);
      int d = sonar.ping_cm();
      if (d == 0) d = MAX_DISTANCE;
      Serial.print(ang);
      Serial.print(",");
      Serial.println(d);
    }
    for (int ang = 180; ang >= 0; ang -= 10) {
      servo.write(ang);
      delay(150);
      int d = sonar.ping_cm();
      if (d == 0) d = MAX_DISTANCE;
      Serial.print(ang);
      Serial.print(",");
      Serial.println(d);
    }
  }

  // === Command Mode ===
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.startsWith("ROTATE")) {
      int angle = cmd.substring(7).toInt();
      rotate(angle);
    } else if (cmd.startsWith("MOVE")) {
      int dist = cmd.substring(5).toInt();
      moveForward(dist);
    }
  }
}

void moveForward(int cm) {
  analogWrite(ENA, 160);
  analogWrite(ENB, 160);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  delay(abs(cm) * MOVE_DELAY_PER_CM);
  stopMotors();
}

void rotate(int deg) {
  analogWrite(ENA, 160);
  analogWrite(ENB, 160);
  if (deg > 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  }
  delay(abs(deg) * ROTATE_DELAY_PER_DEG);
  stopMotors();
}

void stopMotors() {
  analogWrite(ENA, 0); analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
