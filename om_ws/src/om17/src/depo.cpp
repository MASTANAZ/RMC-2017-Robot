#include "depo.h"

#include <iostream>

namespace depo
{
    const int STATE_LOWER = 0;
    const int STATE_RAISE = 1;
    int state = STATE_LOWER;
    float timer = 0.0f;
    bool change_state = false;
    
}

void depo::init(Robot* robot)
{
    std::cout << "DEPO INIT" << std::endl;
    robot->mc2 = -100;
}

void depo::tick(float dt, Robot* robot)
{
    timer += dt;
    if (state == STATE_LOWER)
    {
        robot->mc2 = -50;
        if (timer > 6.0f)
        {
            timer = 0.0f;
            state = STATE_RAISE;
        }
    }
    else if (state == STATE_RAISE)
    {
        robot->mc2 = 50;
        
        if (timer > 6.0f)
        {
            change_state = true;
        }
    }
}

void depo::reset(void)
{
    std::cout << "DEPO RESET" << std::endl;
    change_state = false;
    state = STATE_LOWER;
    timer = 0.0f;
}

bool depo::changeState(void)
{
    return change_state;
}