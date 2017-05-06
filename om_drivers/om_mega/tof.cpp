#include "tof.h"

#include <Arduino.h>
#include <Servo.h>

namespace
{
    const uint8_t PWM_PIN        = 4;
    const uint8_t DEG_PER_SECOND = 50;
    const uint8_t ANGLE_MIN      = 0;
    const uint8_t ANGLE_MAX      = 180;
    const float   ROT_FREQUENCY  = 1.0f / (float)DEG_PER_SECOND;
    Servo sservo;
    float rotation_timer = 0.0f;
    //VL53L0X tf1, tf2;
}

void tof::init(void)
{
    sservo.attach(tof::PWM_PIN);
    sservo.writeMicroseconds(1500);
}

void tof::tick(float dt)
{
    rotation_timer += dt;
    
    if (rotation_timer >= ROT_FREQUENCY)
    {
        
        rotation_timer -= ROT_FREQUENCY;
    }
}