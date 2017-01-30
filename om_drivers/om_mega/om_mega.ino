// om_mega
// 
// CREATED BY HARRIS NEWSTEDER AND BLAKE NAZARIO-CASEY
//
// OPERATION CODE FOR THE ON-BOARD ARDUINO MEGA
// NASA RMC 2017

#include "ir_sense.h"

#include <Servo.h>

IRSense ir;

void setup() {
  Serial.begin(9600);

  ir.setSensorPin(8);
  ir.setServoPin(11);
}

void loop() {
  ir.tick();
}

