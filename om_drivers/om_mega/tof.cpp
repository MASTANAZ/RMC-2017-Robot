#include "tof.h"

#include <Arduino.h>
#include <Servo.h>

#include <Wire.h>
#include <VL53L0X.h>

namespace
{
    const uint8_t  PWM_PIN        = 5;
    const uint8_t  DEG_PER_SECOND = 20;
    const int8_t   ANGLE_MIN      = -30;
    const int8_t   ANGLE_MAX      = 30;
    const int8_t   MS_PER_DEG     = 11;
    const uint16_t MS_CENTER      = 1650;
    const float    ROT_FREQUENCY  = 1.0f / (float)DEG_PER_SECOND;
    //
    const uint8_t  XSHUT_PIN      = 4;
    
    Servo sservo;
    
    float rotation_timer = 0.0f;
    int8_t real_angle = 0;
    int8_t incrementer = 1;
    
    VL53L0X tof_left, tof_right;
}

void tof::init(void)
{
    // shut-down the left TOF sensor
    pinMode(XSHUT_PIN, OUTPUT);
    digitalWrite(XSHUT_PIN, LOW);
    delay(10);
    
    Wire.begin();
    
    //
    tof_right.setAddress(42);
    
    // power-on the left TOF sensor
    // i2c address defaults to 41
    pinMode(XSHUT_PIN, INPUT);
    delay(10);
    
    tof_left.init();
    tof_right.init();
    
    tof_left.setTimeout(500);
    tof_right.setTimeout(500);
    
    tof_left.startContinuous();
    tof_right.startContinuous();
    
    sservo.attach(PWM_PIN);
    sservo.writeMicroseconds(1650);
}

void tof::tick(float dt)
{
    rotation_timer += dt;
    
    if (rotation_timer >= ROT_FREQUENCY)
    {
        real_angle += incrementer;
        
        setAngle(real_angle);
        
        if (real_angle == ANGLE_MAX) incrementer = -1;
        else if (real_angle == ANGLE_MIN) incrementer = 1;
        
        rotation_timer -= ROT_FREQUENCY;
    }
}

void tof::setAngle(int8_t angle)
{
    sservo.writeMicroseconds((int)MS_CENTER + (int)(angle * MS_PER_DEG));
}