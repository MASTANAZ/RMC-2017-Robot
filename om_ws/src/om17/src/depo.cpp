/**
  Created by Blake Nazario-Casey and Harris Newsteder
  
  DESCRIPTION:
    The depo.cpp/h files are used to command motor controllers for 
    deposition buckets.

**/

#include "depo.h"

#include "ros/ros.h"

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
    ROS_INFO_STREAM("DEPO INIT");
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
    ROS_INFO_STREAM("DEPO RESET");
    change_state = false;
    state = STATE_LOWER;
    timer = 0.0f;
}

bool depo::changeState(void)
{
    return change_state;
}