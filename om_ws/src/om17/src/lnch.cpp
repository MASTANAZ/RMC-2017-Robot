#include "lnch.h"

#include <iostream>

namespace lnch
{
    const int STATE_LOWER = 0;
    const int STATE_RAISE = 1;
    int state = STATE_LOWER;
    float timer = 0.0f;
    bool change_state = false;
    
}

void lnch::init(Robot* robot)
{
    std::cout << "LAUNCH INIT" << std::endl;
    robot->mc2 = -100;
}

void lnch::tick(float dt, Robot* robot)
{
    timer += dt;
    
    if (timer > 6.0f)
    {
        change_state = true;
    }
}

void lnch::reset(void)
{
    std::cout << "LAUNCH RESET" << std::endl;
    change_state = false;
    state = STATE_LOWER;
    timer = 0.0f;
}

bool lnch::changeState(void)
{
    return change_state;
}