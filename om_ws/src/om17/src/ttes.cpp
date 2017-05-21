#include "ttes.h"

#include "ros/ros.h"

#include <iostream>

namespace ttes
{
    const int STATE_TRAVEL = 0;
    const int STATE_SURVEY = 1;
    int state = STATE_TRAVEL;
    float timer = 0.0f;
    bool change_state = false;
    float target_y = 1.89f;
}

void ttes::init(Robot* robot)
{
    ROS_INFO_STREAM("TTES INIT");
    change_state = false;
    state = STATE_TRAVEL;
    timer = 0.0f;
}

// We don't need a timeout because AutoControls takes care of that

#warning K-Proportion and K-Derivative need to be determined experimentally
float k_p = 0.0;
float k_d = 0.0;

float last_x_error = 0.0;
float last_y_error = 0.0;
float last_theta_error = 0.0;

void ttes::tick(float dt, Robot* robot)
{   
    // This code may be very very shit, it hasnt been tested. The robot will probalby be extremely jerky.
    
#warning The value for x,y,theta-errors will need to be determined experimentally
    // Error for (x,y,theta)
    //TODO: This error will need to relate to the error from the position of the planned path
    //      relative to the rover.
    float x_error     = 0.75 * robot->x;
    float y_error     = 0.78 * robot->y;
    float theta_error = 0.65 * robot->theta; // This will require a kalman filter to be truly useful
    
    float error_avg = (x_error +y_error + theta_error) / 3;
    
    int motor_speed = k_p * error_avg + k_d * (error_avg - ((last_x_error + last_y_error + last_theta_error)/3));
    
    last_x_error = x_error;
    last_y_error = y_error;
    last_theta_error = theta_error;
    
    robot->mc2 = 75.0 + motor_speed;
    robot->mc1 = 75.0 - motor_speed;
    
    
    
//    robot->mc1 = 100;
//    robot->mc2 = 100;

    
    if (robot->x >= 4.4)
    {
        change_state = true;
    }
}

void ttes::reset(void)
{
    
}

bool ttes::changeState(void)
{
    return change_state;
}
