// ir_sense
// 
// CREATED BY HARRIS NEWSTEDER AND BLAKE NAZARIO-CASEY
//
// 

#ifndef _IR_SENSE_H
#define _IR_SENSE_H

#include <Servo.h>

class IRSense
{
public:
    IRSense();
    ~IRSense();
    
    void setSensorPin(int pin);
    void setServoPin(int pin);
    void tick(void);

private:
    const int        DEG_PER_SECOND = 20;
    const float      ROTATION_TIME  = 1.0f / (float)DEG_PER_SECOND;
    const static int SENSOR_SAMPLES = 50;
    const int        ANGLE_LOWER    = 80;
    const int        ANGLE_UPPER    = 140;

    int sensor_pin_, servo_pin_;
    
    long time_old_, time_current_;
    float rotation_timer_;
    
    Servo servo_;
    int angle_;

    bool forward_;
    
    unsigned short sensor_values_[SENSOR_SAMPLES];
    int sensor_index_;
};

#endif // _IR_SENSE_H
