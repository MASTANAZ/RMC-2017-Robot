#include "lnch.h"

#include "ros/ros.h"

#include <iostream>
#include <string>

// PHOBOS IS IN FRONT
// DEIMOS IS IN BACK

namespace lnch
{
    const int STATE_LOWER = 0;
    const int STATE_RAISE = 1;
    int state = STATE_LOWER;
    float timer = 0.0f;
    bool change_state = false;
    
    int startZone        = 0; // a = 0, b = 1
    int startOrientation = 0; // 0 = North, 1 = South, 2 = East, 3 = West
    int robotID = 0; // 0 = Phobos, 1 = Deimos
    
}

void lnch::init(Robot* robot)
{
    std::cout << "LAUNCH INIT" << std::endl;    
    std::string zone, orientation;
    
    
    
    ros::param::get("/starting_zone", zone);
    ros::param::get("/starting_orientation", orientation);
    
    #warning This may not be an int. May need to use string and cast to int
    ros::param::get("id", robotID);
    
    
    // Check zone
    // Compare function returns 0 if they are the same, hence the preceeding '!'
    if (!zone.compare("a"))
        startZone = 0;
    
    else if (!zone.compare("b"))
        startZone = 1;
        
    else 
        ROS_FATAL("Starting zone does not equal 'a' or 'b'");
    
    
    // Check orientation
    if (!orientation.compare("n"))
        startOrientation = 0;
    
    else if (!orientation.compare("e"))
        startOrientation = 1;
    
    else if (!orientation.compare("s"))
        startOrientation = 2;
    
    else if (!orientation.compare("w"))
        startOrientation = 3;

    else 
        ROS_FATAL("Starting orientation does not equal 'n', 'e', 's', or 'w'.");
    
        
}

void lnch::tick(float dt, Robot* robot)
{
    timer += dt;
    
    // Robot is phobos (in front)
    if (robotID == 0)
    {
/******************************************************************************
    ZONE A
******************************************************************************/
        // In zone A, Facing North
        // Begin turn right
        if (startZone == 0 && startOrientation == 0)
        {
            // wait for deimos to back up
            if (timer >= 1) 
            {
                robot->mc1 = 100;
                robot->mc2 = -100; 
            
                // Start to travel in positive-y
                if (timer >= 2) 
                {   // mc1 is already at 100
                    robot->mc2 = 100;
                    
                    // turn right
                    if (timer >= 5) 
                    {
                        robot->mc2 = -100;
                        
                        // Change state
                        if (timer >= 6) 
                        {
                            change_state = true;
                        }//End change state
                    }//End turn right
                }//End travel in positive-y
            }// End wait for Deimos
        }
        
        // In zone A, Facing South
        else if (startZone == 0 && startOrientation == 1)
        {
            // Already facing the direction we need to travel,
            // and Deimos is behind phobos. So, just travel forward. 
            change_state = true;
        }
        
        // In zone A, Facing East
        else if (startZone == 0 && startOrientation == 2)
        {
            // Begin to travel in positive y-direction
            robot->mc1 = 100; 
            robot->mc2 = 100; 
            
            // Begin to turn right
            if (timer >= 3)
            {   // mc1 is already at 100
                robot->mc2 = -100;
                
                // Change state
                if (timer >= 4)
                {
                    change_state = true;
                }// End change state
            } // End turn right
        }
        
        // In zone A, Facing West
        else if (startZone == 0 && startOrientation == 3)
        {
            // wait for deimos to back up
            if (timer >= 1) 
            {
                // Begin turn left
                robot->mc1 = -100;
                robot->mc2 = 100;
                
                // Change state
                if (timer >= 2)
                {
                    change_state = true;
                } //End change state
            }//End wait for deimos
        }
        
/******************************************************************************
    ZONE B
******************************************************************************/
        // In zone B, Facing North
        else if (startZone == 1 && startOrientation == 0)
        {
            // wait for deimos to back up
            if (timer >= 1) 
            {
                robot->mc1 = -100;
                robot->mc2 = 100; 
            
                // Start to travel in positive-y
                if (timer >= 2) 
                {   // mc2 is already at 100
                    robot->mc1 = 100;
                    
                    // turn left
                    if (timer >= 5) 
                    {
                        robot->mc1 = -100;
                        
                        // Change state
                        if (timer >= 6) 
                        {
                            change_state = true;
                        }//End change state
                    }//End turn right
                }//End travel in positive-y
            }// End wait for Deimos
        }
        
        // In zone B, Facing South
        else if (startZone == 1 && startOrientation == 1)
        {
            // Already facing the direction we need to travel,
            // and Deimos is behind phobos. So, just travel forward. 
            change_state = true;
        }
        
        // In zone B, Facing East
        else if (startZone == 0 && startOrientation == 2)
        {
            // wait for deimos to back up
            if (timer >= 1) 
            {
                // Begin turn right
                robot->mc1 = 100;
                robot->mc2 = -100;
                
                // Change state
                if (timer >= 2)
                {
                    change_state = true;
                } //End change state
            }//End wait for deimos
        }
        
        // In zone A, Facing West
        else if (startZone == 0 && startOrientation == 3)
        {
            // Begin to travel in negative y-direction
            robot->mc1 = 100; 
            robot->mc2 = 100; 
            
            // Begin to turn right
            if (timer >= 3)
            {   // mc1 is already at 100
                robot->mc2 = -100;
                
                // Change state
                if (timer >= 4)
                {
                    change_state = true;
                }// End change state
            } // End turn right
        }
        
        else {
            ROS_FATAL("INVALID STARTING POSITION");
        }
    }
    
    
/******************************************************************************
    END PHOBOS
    BEGIN DEIMOS
******************************************************************************/

    
    // Robot is deimos (in back)
    else 
    {
        
    }
    
    /**
    if (timer > 6.0f)
    {
        change_state = true;
    }
    **/
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