// tof.h
// 
// CREATED BY HARRIS NEWSTEDER
//

#ifndef _TOF_H
#define _TOF_H

#include <Servo.h>
#include <VL53L0X.h>

class TOF
{
public:
    TOF(void);
    ~TOF(void);

    void tick(void);
    void setServoPin(int pin);
    void setId(int id);
private:
    int id_;
    Servo servo_;
    VL53L0X sensor_;
    long time_old_, time_current_;
};

#endif // _TOF_H