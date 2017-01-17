// IRsensor_servo
// 
// Created by Harris Newsteder / Blake Nazario-Casey
//
// Arduino code for the IR object detector and it's servo. 

#include <Servo.h>

Servo sensorServo;

int angle = 40;      // Starting angle (degrees)
int sensorpin = 0;   // IR detector pin
int val = 0;         // IR Detector value

long old, current;

// Servo parameters
const int DEG_PER_SECOND = 20;
const float ROT_TIME = 1.0f / (float)DEG_PER_SECOND;


// Timer for servo rotation
float rotationTimer = 0;

bool forward = true;

// Flag to lock the server if the sensor detects an object
bool lock = false;

// Circular array of IR values aggregated to average in order to flatten the values
unsigned short sensorValues[50];
int sensorIndex = 0;


// Setup control loop parameters
void setup() {
  Serial.begin(9600);
  sensorServo.attach(11);  // attaches the servo on pin 9 to the servo object
  old = millis();

  for (int i = 0; i < 50; ++i) {
    sensorValues[i] = 0;
  }
}


// Begin control loop
void loop() {
  current = millis();
  float dt = (float)(current - old) / 1000.0f;
  old = current;

  rotationTimer += dt;

  
  sensor();

  if (rotationTimer > ROT_TIME) {
    float avg = 0;

    // Insert sensor values into average
    for (int i = 0; i < 50; ++i) {
      avg += (float)sensorValues[i]; 
    }

    // Calculate the average
    avg = avg / 50.0f;

    float dist = pow((4187.8 / avg), 1.106);

    if (dist < 30) {
      lock = true;
      Serial.println(dist);
    } else {
      lock = false;
    }

    if (lock) return;
    
    if (forward) {
      angle++;
      if (angle == 110) forward = false;
    } else {
      angle--;
      if (angle == 40) forward = true;
    }

    sensorServo.write(angle);
    rotationTimer = 0;
  }
}

void sensor(){
  val = analogRead(sensorpin);
  sensorValues[sensorIndex] = val;
  sensorIndex++;
  if (sensorIndex == 50) sensorIndex = 0;
}

