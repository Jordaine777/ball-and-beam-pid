#include <Servo.h>

Servo myServo;

// --- pins ---
const int servoPin = 9;      // change to your servo signal pin
const int trigPin = 10;      // change to your ultrasonic trig pin
const int echoPin = 11;      // change to your ultrasonic echo pin

// --- setpoint & gains ---
float target = 14;         // where on the beam we want the car (your units)
float kp = 1.5;
float ki = 0;//0.2;
float kd = 0; //0.21;

// --- servo geometry ---
const int CENTER = 95;       // when beam is flat
const int SERVO_MIN = 87;    // pulled in from 73
const int SERVO_MAX = 103;   // pulled in from 117

// --- loop state ---
float error_sum = 0;
float previous_error = 0;
float last_good = 18.0;      // start at target so nothing jumps on boot
unsigned long last_time = 0;
bool saturated = false;

void setup() {
  Serial.begin(9600);
  myServo.attach(servoPin);
  myServo.write(CENTER);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  last_time = millis();
  delay(500);
}

float readDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);  // timeout so it can't hang
  float dist = duration * 0.0343 / 2.0;
  return dist;
}

void loop() {
  // --- real dt ---
  unsigned long now = millis();
  float dt = (now - last_time) / 1000.0;
  last_time = now;
  if (dt <= 0) return;   // guard against a zero interval

  // --- read sensor, reject bad readings ---
  float distance = readDistance();
  if (distance >= 3 && distance <= 24) {
    last_good = distance;
  }
  float measured = last_good;

  // --- PID ---
  float error = target - measured;
  float P = kp * error;

  if (!saturated) {
    error_sum = error_sum + error * dt;
  }
  float I = ki * error_sum;

  float D = kd * (error - previous_error) / dt;

  float output = P + I + D;

  // --- map to servo around your real center ---
  float angle = CENTER - output;   // may need CENTER + output

  // --- clamp + track saturation for anti-windup ---
  saturated = false;
  if (angle > SERVO_MAX) { angle = SERVO_MAX; saturated = true; }
  if (angle < SERVO_MIN) { angle = SERVO_MIN; saturated = true; }

  myServo.write((int)angle);
  previous_error = error;

  // --- serial: your plot ---
  //Serial.print("dist: "); Serial.println(measured);
  //Serial.print("  err: ");  Serial.print(error);
  //Serial.print("  ang: ");  Serial.println((int)angle);
  Serial.print("distance:");
  Serial.print(measured);
  Serial.print(",target:");
  Serial.println(target);

  //delay(20);   // ~50 Hz loop
}