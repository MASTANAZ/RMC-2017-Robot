/**
  Created by Blake Nazario-Casey and Harris Newsteder
  
  DESCRIPTION:
    The depo.cpp/h files are used to command motor controllers for 
    deposition buckets.

**/

#include "depo.h"

#include "ros/ros.h"

#include <iostream>
#include <string>

namespace depo
{
    bool change_state = false;
}

void depo::init(Robot* robot)
{
    ROS_INFO_STREAM("DEPO INIT");
}

void depo::tick(float dt, Robot* robot)
{

}

void depo::reset(void)
{
    ROS_INFO_STREAM("DEPO RESET");
    change_state = false;
}

bool depo::changeState(void)
{
    return change_state;
}