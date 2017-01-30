#include "ir_sense.h"

#include <Arduino.h>
#include <Servo.h>

#include <math.h>

IRSense::IRSense() {
    sensor_pin_ = -1;
    servo_pin_ = -1;
    
    time_old_ = 0;
    time_current_ = 0;
    
    rotation_timer_ = 0;
    
    angle_ = ANGLE_LOWER;

    forward_ = true;
    
    sensor_index_ = 0;
    
    for (unsigned i = 0; i < SENSOR_SAMPLES; ++i) {
        sensor_values_[i] = 0;
    }
}

IRSense::~IRSense() {
    
}

void IRSense::setSensorPin(int pin) {
    sensor_pin_ = pin;
}

void IRSense::setServoPin(int pin) {
    servo_pin_ = pin;
    servo_.attach(pin);
}

void IRSense::tick(void) {
    // calculate the time (in seconds) since the last call to tick()
    time_current_ = millis();
    float dt = (float)(time_current_ - time_old_) / 1000.0f;
    time_old_ = time_current_;
    
    rotation_timer_ += dt;
    
    sensor_values_[sensor_index_] = analogRead(sensor_pin_);
    sensor_index_++;
    if (sensor_index_ == SENSOR_SAMPLES) sensor_index_ = 0;
    
    if (rotation_timer_ > ROTATION_TIME) {
        float avg = 0.0f;

        for (int i = 0; i < SENSOR_SAMPLES; ++i) {
            avg += (float)sensor_values_[i]; 
        }
        
        avg = avg / (float)SENSOR_SAMPLES;
        
        float dist = pow((4187.8 / avg), 1.106);
        
        Serial.println(dist);
        
        if (forward_) {
            ++angle_;
            if (angle_ == ANGLE_UPPER) forward_ = false;
        } else {
            --angle_;
            if (angle_ == ANGLE_LOWER) forward_ = true;
        }
        
        servo_.write(angle_);
        
        rotation_timer_ -= ROTATION_TIME;
    }
}