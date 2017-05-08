#pragma once

#include "robot.h"

namespace lnch
{
    void init(Robot* robot);
    void tick(float dt, Robot* robot);
    void reset(void);
    bool changeState(void);
}