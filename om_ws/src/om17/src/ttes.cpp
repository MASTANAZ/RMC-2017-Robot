#include "ttes.h"

#include <iostream>

namespace ttes
{
    const int STATE_TRAVEL = 0;
    const int STATE_SURVEY = 1;
    int state = STATE_TRAVEL;
    float timer = 0.0f;
    bool change_state = false;
}

void ttes::init(Robot* robot)
{
    std::cout << "TTES INIT" << std::endl;
}

void ttes::tick(float dt, Robot* robot)
{
    timer += dt;
    
    if (timer > 5.0f)
    {
        change_state = true;
    }
}

void ttes::reset(void)
{
    std::cout << "TTES RESET" << std::endl;
    change_state = false;
    state = STATE_TRAVEL;
    timer = 0.0f;
}

bool ttes::changeState(void)
{
    return change_state;
}