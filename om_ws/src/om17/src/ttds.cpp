#include "ttds.h"

#include "ros/ros.h"

#include <iostream>

namespace ttds
{
    const int STATE_TRAVEL = 0;
    const int STATE_SURVEY = 1;
    int state = STATE_TRAVEL;
    float timer = 0.0f;
    bool change_state = false;
}

void ttds::init(Robot* robot)
{
    ROS_INFO_STREAM("TTDS INIT");
}

void ttds::tick(float dt, Robot* robot)
{   
    bool lost_tracking;
    ros::param::get("lost_tracking", lost_tracking);

    if (lost_tracking)
    {
        robot->mc1 = 0;
        robot->mc2 = 0;
    }
    else
    {
        robot->mc1 = -100;
        robot->mc2 = -100;
    }
    
    if (robot->x < 1)
    {
        change_state = true;
    }
}

void ttds::reset(void)
{
    ROS_INFO_STREAM("TTDS RESET");
    change_state = false;
    state = STATE_TRAVEL;
    timer = 0.0f;
}

bool ttds::changeState(void)
{
    return change_state;
}