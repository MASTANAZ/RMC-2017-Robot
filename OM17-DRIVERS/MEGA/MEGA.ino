#include <Servo.h>

const int SCAN_LOWER = 80;
const int SCAN_UPPER = 140;

Servo sensorServo;

int angle = SCAN_LOWER;
int sensorpin = 0;
int val = 0;

long old, current;

const int DEG_PER_SECOND = 40;
const float ROT_TIME = 1.0f / (float)DEG_PER_SECOND;

float rotationTimer = 0;

bool forward = true;

void setup() {
  Serial.begin(9600);
  sensorServo.attach(11);  // attaches the servo on pin 9 to the servo object
  old = millis();
}

void loop() {
  current = millis();
  float dt = (float)(current - old) / 1000.0f;
  old = current;

  rotationTimer += dt;

  if (rotationTimer > ROT_TIME) {
    if (forward) {
      angle++;
      if (angle == SCAN_UPPER) forward = false;
    } else {
      angle--;
      if (angle == SCAN_LOWER) forward = true;
    }

    sensorServo.write(angle);
    rotationTimer = 0;
  }
}

