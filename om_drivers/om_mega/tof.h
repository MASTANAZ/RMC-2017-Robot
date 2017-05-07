#pragma once

#include <Arduino.h>

namespace tof
{
    void init(void);
    void tick(float dt);
    void setAngle(int8_t angle);
}