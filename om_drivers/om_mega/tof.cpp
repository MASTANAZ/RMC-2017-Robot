// CREATED BY HARRIS NEWSTEDER
//
// DESCRIPTION:
//   
//   

////////////////////////////////////////////////////////////////////////////////
// INCLUDES
////////////////////////////////////////////////////////////////////////////////

#include "tof.h"

#include <Arduino.h>
#include <Servo.h>

#include <math.h>

////////////////////////////////////////////////////////////////////////////////
// PUBLIC FUNCTION DEFINITIONS
////////////////////////////////////////////////////////////////////////////////

TOF::TOF()
{
    id_ = -1;
    time_old_ = 0;
    time_current_ = 0;
    
    servo_ = NULL;
    sensor_ = NULL;
}

TOF::~TOF()
{
    
}

void TOF::setServoPin(int pin)
{
    servo_.attach(pin);
}

void TOF::tick(void)
{
    // calculate the time (in seconds) since the last call to tick()
    time_current_ = millis();
    float dt = (float)(time_current_ - time_old_) / 1000.0f;
    time_old_ = time_current_;
}