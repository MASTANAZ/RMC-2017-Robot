#include "lnch.h"

#include "ros/ros.h"

#include <iostream>
#include <string>
#include <vector>

// PHOBOS IS IN FRONT
// DEIMOS IS IN BACK

// void namespace
namespace
{
const int DRACTION_DRIVE_FORWARD  = 0;
const int DRACTION_DRIVE_BACKWARD = 1;
const int DRACTION_TURN_RIGHT     = 2;
const int DRACTION_TURN_LEFT      = 3;
const int DRACTION_WAIT           = 4;

// TUNE THIS
const float DRACTION_DURATION_TURN_90 = 2.0f;

// dead reckoning action
struct DRAction
{
    float timer = 0.0f;
    float duration = 0.0f;
    int mc1_set = 0;
    int mc2_set = 0;
    
    DRAction(int type, float action_duration = 0.0f)
    {
        duration = action_duration;
        
        switch (type)
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
        case DRACTION_WAIT:
            mc1_set = 0;
            mc2_set = 0;
        default:
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
    
    int start_zone        = 0; // a = 0, b = 1
    int start_orientation = 0; // 0 = North, 1 = South, 2 = East, 3 = West
    int robot_id          = 0; // 0 = Phobos, 1 = Deimos
    
    int current_action    = 0;
    
    // ACTIONS TO TAKE FOR THE LAUNCH
    std::vector<DRAction> launch_actions;
}

void lnch::init(Robot* robot)
{
    std::cout << "LNCH INIT" << std::endl;
    
    ros::param::get("/starting_zone", start_zone);
    ros::param::get("/starting_orientation", start_orientation);
    #warning This may not be an int. May need to use string and cast to int
    ros::param::get("id", robot_id);
    
    if (robot_id == ID_PHOBOS)
    {
        if (start_zone == ZONE_A)
        {
            if (start_orientation == ORIENTATION_NORTH)
            {
                launch_actions.push_back(DRAction(DRAction::WAIT, 1.0f));
                launch_actions.push_back(DRAction(DRAction::DRIVE_BACKWARD, 1.0));
                launch_actions.push_back(DRAction(DRAction::TURN_RIGHT));
                launch_actions.push_back(DRAction(DRAction::DRIVE_FORWARD, 2.0));
                launch_actions.push_back(DRAction(DRAction::TURN_RIGHT));
            }
            else if (start_orientation == ORIENTATION_SOUTH)
            {
                
            }
            else if (start_orientation == ORIENTATION_EAST)
            {
                
            }
            else if (start_orientation == ORIENTATION_WEST)
            {
                
            }
        }
        else if (start_zone == ZONE_B)
        {
            if (start_orientation == ORIENTATION_NORTH)
            {
                launch_actions.push_back(DRAction(DRAction::WAIT, 1.0f));
                launch_actions.push_back(DRAction(DRAction::DRIVE_BACKWARD, 1.0));
                launch_actions.push_back(DRAction(DRAction::TURN_RIGHT));
                launch_actions.push_back(DRAction(DRAction::DRIVE_FORWARD, 2.0));
                launch_actions.push_back(DRAction(DRAction::TURN_RIGHT));
            }
            else if (start_orientation == ORIENTATION_SOUTH)
            {
                
            }
            else if (start_orientation == ORIENTATION_EAST)
            {
                
            }
            else if (start_orientation == ORIENTATION_WEST)
            {
                
            }
        }
    }
    else if (robot_id == ID_DEIMOS)
    {
        if (start_zone == ZONE_A)
        {
            if (start_orientation == ORIENTATION_NORTH)
            {
                launch_actions.push_back(DRAction(DRAction::WAIT, 1.0f));
                launch_actions.push_back(DRAction(DRAction::DRIVE_BACKWARD, 1.0));
                launch_actions.push_back(DRAction(DRAction::TURN_RIGHT));
                launch_actions.push_back(DRAction(DRAction::DRIVE_FORWARD, 2.0));
                launch_actions.push_back(DRAction(DRAction::TURN_RIGHT));
            }
            else if (start_orientation == ORIENTATION_SOUTH)
            {
                
            }
            else if (start_orientation == ORIENTATION_EAST)
            {
                
            }
            else if (start_orientation == ORIENTATION_WEST)
            {
                
            }
        }
        else if (start_zone == ZONE_B)
        {
            if (start_orientation == ORIENTATION_NORTH)
            {
                launch_actions.push_back(DRAction(DRAction::WAIT, 1.0f));
                launch_actions.push_back(DRAction(DRAction::DRIVE_BACKWARD, 1.0));
                launch_actions.push_back(DRAction(DRAction::TURN_RIGHT));
                launch_actions.push_back(DRAction(DRAction::DRIVE_FORWARD, 2.0));
                launch_actions.push_back(DRAction(DRAction::TURN_RIGHT));
            }
            else if (start_orientation == ORIENTATION_SOUTH)
            {
                
            }
            else if (start_orientation == ORIENTATION_EAST)
            {
                
            }
            else if (start_orientation == ORIENTATION_WEST)
            {
                
            }
        }
    }
}

void lnch::tick(float dt, Robot* robot)
{
    DRAction current;
    
    current = launch_actions.at(current_action);
    
    current.timer += dt;
    
    if (current.timer >= current.duration)
    {
        current_action++;
        if (current_action == launch_actions.size())
        {
            change_state = true;
        }
        else
        {
            launch_actions.at(current_action).setMCValues(robot);
        }
    }
}

void lnch::reset(void)
{
    std::cout << "LNCH RESET" << std::endl;
    change_state = false;
    timer = 0.0f;
}

bool lnch::changeState(void)
{
    return change_state;
}
