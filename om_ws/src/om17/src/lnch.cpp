#include "lnch.h"

#include "ros/ros.h"

#include <iostream>
#include <vector>

// PHOBOS IS IN FRONT
// DEIMOS IS IN BACK

// void namespace
namespace
{

//
const int DRACTION_DRIVE_FORWARD  = 0;
const int DRACTION_DRIVE_BACKWARD = 1;
const int DRACTION_TURN_RIGHT     = 2;
const int DRACTION_TURN_LEFT      = 3;
const int DRACTION_WAIT           = 4;

#warning The DRACTION_DURATION_TURN_90 should be accurately tuned before being pushed to production.
const float DRACTION_DURATION_TURN_90 = 2.0f;

// DRAction = dead-reckoning action
struct DRAction
{
    // 
    float timer = 0.0f;
    float duration = 0.0f;
    int mc1_set = 0;
    int mc2_set = 0;
    int type = -1;
    
    DRAction(int action_type, float action_duration = 0.0f)
    {
        duration = action_duration;
        type = action_type;
        switch (action_type)
        {
        case DRACTION_DRIVE_FORWARD:
            mc1_set = 100;
            mc2_set = 100;
            break;
        case DRACTION_DRIVE_BACKWARD:
            mc1_set = -100;
            mc2_set = -100;
            break;
        case DRACTION_TURN_RIGHT:
            mc1_set = 100;
            mc2_set = -100;
            duration = DRACTION_DURATION_TURN_90;
            break;
        case DRACTION_TURN_LEFT:
            mc1_set = -100;
            mc2_set = 100;
            duration = DRACTION_DURATION_TURN_90;
            break;
        default:
        case DRACTION_WAIT:
            mc1_set = 0;
            mc2_set = 0;
            break;
        }
    }
    
    void setMCValues(Robot* robot)
    {
        robot->mc1 = mc1_set;
        robot->mc2 = mc2_set;
    }
};
}

namespace lnch
{
    const int ZONE_A = 0;
    const int ZONE_B = 1;
    
    const int ORIENTATION_NORTH = 0;
    const int ORIENTATION_SOUTH = 1;
    const int ORIENTATION_EAST  = 2;
    const int ORIENTATION_WEST  = 3;
    
    const int ID_PHOBOS = 0;
    const int ID_DEIMOS = 1;
    
    float timer = 0.0f;
    bool change_state = false;
    bool first_action_taken = false;
    
    int start_zone        = 0; // a = 0, b = 1
    int start_orientation = 0; // 0 = North, 1 = South, 2 = East, 3 = West
    int robot_id          = 0; // 0 = Phobos, 1 = Deimos
    
    int current_action    = 0;
    
    // ACTIONS TO TAKE FOR THE LAUNCH
    std::vector<DRAction> launch_actions;
}

void lnch::init(Robot* robot)
{
    ROS_INFO_STREAM("LNCH INIT");
    
    ros::param::get("/starting_zone", start_zone);
    ros::param::get("/starting_orientation", start_orientation);
    ros::param::get("id", robot_id);
    
    ROS_INFO_STREAM("starting_zone = " << start_zone);
    ROS_INFO_STREAM("starting_orientation = " << start_orientation);
    ROS_INFO_STREAM("robot id = " << robot_id);

    if (robot_id == ID_PHOBOS)
    {
        if (start_zone == ZONE_A)
        {
            if (start_orientation == ORIENTATION_NORTH)
            {
                launch_actions.push_back(DRAction(DRACTION_WAIT, 1.0f));
                launch_actions.push_back(DRAction(DRACTION_DRIVE_BACKWARD, 1.0));
                launch_actions.push_back(DRAction(DRACTION_TURN_RIGHT));
                launch_actions.push_back(DRAction(DRACTION_DRIVE_FORWARD, 2.0));
                launch_actions.push_back(DRAction(DRACTION_TURN_RIGHT));
            }
            else if (start_orientation == ORIENTATION_SOUTH)
            {
                // Phobos will be moving first in this position, no need to wait
                //launch_actions.push_back(DRAction(DRACTION_WAIT, 1.0f));
                launch_actions.push_back(DRAction(DRACTION_DRIVE_FORWARD, 1.0));
            }
            else if (start_orientation == ORIENTATION_EAST)
            {
                // Phobos will be moving first in this position, no need to wait
                // launch_actions.push_back(DRAction(DRACTION_WAIT, 1.0f));
                launch_actions.push_back(DRAction(DRACTION_DRIVE_FORWARD, 3.0));
                launch_actions.push_back(DRAction(DRACTION_TURN_RIGHT));
            }
            else if (start_orientation == ORIENTATION_WEST)
            {
                // Phobos will be taking the Zone A lane here.
                launch_actions.push_back(DRAction(DRACTION_WAIT, 1.0f));
                launch_actions.push_back(DRAction(DRACTION_TURN_LEFT);                
            }
        }
        else if (start_zone == ZONE_B)
        {
            if (start_orientation == ORIENTATION_NORTH)
            {
                // Phobos should take the B lane here, so it will just need to turn
                // 180 degrees
                // Waiting 1 second longer to ensure deimos is out of the way enough
                launch_actions.push_back(DRAction(DRACTION_WAIT, 2.0f));
                launch_actions.push_back(DRAction(DRACTION_TURN_LEFT));
                launch_actions.push_back(DRAction(DRACTION_TURN_LEFT));
                
            }
            else if (start_orientation == ORIENTATION_SOUTH)
            {
                // Phobos will be moving first in this position, no need to wait
                //launch_actions.push_back(DRAction(DRACTION_WAIT, 1.0f));
                launch_actions.push_back(DRAction(DRACTION_DRIVE_FORWARD, 1.0));
            }
            else if (start_orientation == ORIENTATION_EAST)
            {
                // Phobos will wait for Deimos and then turn 90 degrees clockwise
                // to face south
                launch_actions.push_back(DRAction(DRACTION_WAIT, 1.0f));
                launch_actions.push_back(DRAction(DRACTION_TURN_RIGHT));
                
            }
            else if (start_orientation == ORIENTATION_WEST)
            {
                // Phobos will be moving first in this position, no need to wait
                // launch_actions.push_back(DRAction(DRACTION_WAIT, 1.0f));
                launch_actions.push_back(DRAction(DRACTION_DRIVE_FORWARD, 3.0));
                launch_actions.push_back(DRAction(DRACTION_TURN_LEFT));
            }
        }
    }
    else if (robot_id == ID_DEIMOS)
    {
        if (start_zone == ZONE_A)
        {
            if (start_orientation == ORIENTATION_NORTH)
            {
                // Deimos needs to backup and turn 180 degrees
                launch_actions.push_back(DRAction(DRACTION_WAIT, 1.0f));
                launch_actions.push_back(DRAction(DRACTION_DRIVE_BACKWARD, 1.0));
                launch_actions.push_back(DRAction(DRACTION_TURN_RIGHT));
                launch_actions.push_back(DRAction(DRACTION_TURN_RIGHT));
            }
            else if (start_orientation == ORIENTATION_SOUTH)
            {
                // Deimos will take the B lane here
                launch_actions.push_back(DRAction(DRACTION_WAIT, 1.0f));
                launch_actions.push_back(DRAction(DRACTION_DRIVE_FORWARD, 1.0));
                launch_actions.push_back(DRAction(DRACTION_TURN_LEFT));
                launch_actions.push_back(DRAction(DRACTION_DRIVE_FORWARD, 3.0));
                launch_actions.push_back(DRACTION_TURN_RIGHT));
            }
            else if (start_orientation == ORIENTATION_EAST)
            {
                // Deimos will take the A lane here
                launch_actions.push_back(DRAction(DRACTION_WAIT, 1.0f));
                launch_actions.push_back(DRAction(DRACTION_TURN_RIGHT));
            }
            else if (start_orientation == ORIENTATION_WEST)
            {
                // Deimos will take the B lane here, and moving first
                //launch_actions.push_back(DRAction(DRACTION_WAIT, 1.0f));
                launch_actions.push_back(DRAction(DRACTION_DRIVE_BACKWARD, 3.0));
                launch_actions.push_back(DRAction(DRACTION_TURN_LEFT));
            }
        }
        else if (start_zone == ZONE_B)
        {
            if (start_orientation == ORIENTATION_NORTH)
            {
                // Deimos will take the A lane here, and move first
                // launch_actions.push_back(DRAction(DRACTION_WAIT, 1.0f));
                launch_actions.push_back(DRAction(DRACTION_DRIVE_BACKWARD, 1.0));
                launch_actions.push_back(DRAction(DRACTION_TURN_LEFT));
                launch_actions.push_back(DRAction(DRACTION_DRIVE_FORWARD, 3.0));
                launch_actions.push_back(DRAction(DRACTION_TURN_LEFT));
            }
            else if (start_orientation == ORIENTATION_SOUTH)
            {
                // Deimos will take the A lane here
                launch_actions.push_back(DRAction(DRACTION_WAIT, 1.0f));
                launch_actions.push_back(DRAction(DRACTION_TURN_RIGHT));
                launch_actions.push_back(DRACTION_DRIVE_FORWARD, 3.0));
                launch_actions.push_back(DRAction(DRACTION_TURN_LEFT));
            }
            else if (start_orientation == ORIENTATION_EAST)
            {
                // Deimos will take the B lane here, and moving first
                //launch_actions.push_back(DRAction(DRACTION_WAIT, 1.0f));
                launch_actions.push_back(DRAction(DRACTION_DRIVE_BACKWARD, 3.0));
                launch_actions.push_back(DRAction(DRACTION_TURN_Right));
            }
            else if (start_orientation == ORIENTATION_WEST)
            {
                // Deimos will be taking the Zone B lane here.
                launch_actions.push_back(DRAction(DRACTION_WAIT, 1.0f));
                launch_actions.push_back(DRAction(DRACTION_TURN_LEFT);
            }
        }
    }
}

void lnch::tick(float dt, Robot* robot)
{
    DRAction& current = launch_actions.at(current_action);
    
    if (!first_action_taken)
    {
        current.setMCValues(robot);
        first_action_taken = true;
    }
    
    current.timer += dt;
    
    // the time is up for the current action in the sequence, move to the next
    // action
    if (current.timer >= current.duration)
    {
        current_action++;
        
        // there is no more actions to take; change state
        if (current_action == launch_actions.size())
        {
            change_state = true;
        }
        // set the motor controller values accordingly for the next action
        else
        {
            launch_actions.at(current_action).setMCValues(robot);
        }
    }
}

void lnch::reset(void)
{
    ROS_INFO_STREAM("LNCH RESET");
    change_state = false;
    first_action_taken = false;
    timer = 0.0f;
}

bool lnch::changeState(void)
{
    return change_state;
}
