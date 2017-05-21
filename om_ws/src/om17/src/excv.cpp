#include "excv.h"

#include "ros/ros.h"

#include <iostream>

namespace excv
{
    const int STATE_LOWER = 0;
    const int STATE_RAISE = 1;
    int state = STATE_LOWER;
    float timer = 0.0f;
    bool change_state = false;
    
}

void excv::init(Robot* robot)
{
    ROS_INFO_STREAM("EXCV INIT");
    robot->mc2 = 0;
}

void excv::tick(float dt, Robot* robot)
{
    timer += dt;
    
    if (timer > 6.0f)
    {
        change_state = true;
    }
}

void excv::reset(void)
{
    ROS_INFO_STREAM("EXCV RESET");
    change_state = false;
    state = STATE_LOWER;
    timer = 0.0f;
}

bool excv::changeState(void)
{
    return change_state;
}