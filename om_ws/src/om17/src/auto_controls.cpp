// CREATED BY HARRIS NEWSTEDER AND BLAKE NAZARIO-CASEY
//
// DESCRIPTION:
//

////////////////////////////////////////////////////////////////////////////////
// INCLUDES
////////////////////////////////////////////////////////////////////////////////

#include <fstream>
#include <iostream>
#include <cstdlib>
#include <chrono>
#include <vector>
#include <thread>
#include <algorithm>

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"

////////////////////////////////////////////////////////////////////////////////
// CONSTANTS
////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////
// NODE VARIABLES
////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////
// FUNCTION DECLARATIONS
////////////////////////////////////////////////////////////////////////////////

void init(void);
void tick(void);
void cleanup(void);

////////////////////////////////////////////////////////////////////////////////
// ENTRY POINT
////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
    // ROS initialization
    ros::init(argc, argv, "auto_controls");
    ros::NodeHandle node_handle;
    ros::Rate loop_rate(3);
    
    init();
    
    while (ros::ok())
    {
        tick();
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    cleanup();
    
    return EXIT_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////
// FUNCTION DEFINITIONS
////////////////////////////////////////////////////////////////////////////////

void init(void)
{
    
}

void tick(void)
{
    
}

void cleanup(void)
{
    
}
