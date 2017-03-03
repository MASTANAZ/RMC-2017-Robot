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
#include <stdlib.h>

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"
#include "dstar.h"
#include "om17/CellCost.h"

////////////////////////////////////////////////////////////////////////////////
// CONSTANTS
////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////
// NODE VARIABLES
////////////////////////////////////////////////////////////////////////////////

// all topics that the auto_controls node subscribes to
ros::Subscriber pose_sub, world_cost_sub;

// all topics that the auto_controls node publishes to
ros::Publisher lcv_pub, rcv_pub;
ros::Publisher cstate_pub;

////////////////////////////////////////////////////////////////////////////////
// FUNCTION DECLARATIONS
////////////////////////////////////////////////////////////////////////////////

void init(ros::NodeHandle &node_handle);
void tick(void);
void cleanup(void);
void worldCostCallback(const om17::CellCost::ConstPtr& msg);
void pathTest();

////////////////////////////////////////////////////////////////////////////////
// ENTRY POINT
////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
    // ROS initialization
    ros::init(argc, argv, "auto_controls");
    ros::NodeHandle node_handle;
    ros::Rate loop_rate(30);
    
    std_msgs::Int8 tmp;
    tmp.data = 8;

    init(node_handle);
    
    while (ros::ok())
    {
        tick();
        cstate_pub.publish(tmp);
        ros::spinOnce();
        loop_rate.sleep();
    }

    cleanup();
    
    return EXIT_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////
// FUNCTION DEFINITIONS
////////////////////////////////////////////////////////////////////////////////

void init(ros::NodeHandle &node_handle)
{
    lcv_pub = node_handle.advertise<std_msgs::Int16>("lcv", 100);
	rcv_pub = node_handle.advertise<std_msgs::Int16>("rcv", 100);
	cstate_pub = node_handle.advertise<std_msgs::Int8>("control_state", 100);

    world_cost_sub = node_handle.subscribe("/world_cost", 10, worldCostCallback);

    pathTest();
}

void worldCostCallback(const om17::CellCost::ConstPtr& msg)
{
    std::cout << msg->x << std::endl;
    std::cout << msg->y << std::endl;
    std::cout << msg->cost << std::endl;
    std::cout << "----------" << std::endl;
}

void pathTest()
{
    Dstar *dstar = new Dstar();
    list <state> mypath;

    dstar->init(0,0,10,5);         // set start to (0,0) and goal to (10,5)
    dstar->updateCell(3,4,-1);     // set cell (3,4) to be non traversable
    dstar->updateCell(2,2,42.432); // set set (2,2) to have cost 42.432

    dstar->replan();               // plan a path
    mypath = dstar->getPath();     // retrieve path

    dstar->updateStart(10,2);      // move start to (10,2)
    dstar->replan();               // plan a path
    mypath = dstar->getPath();     // retrieve path

    dstar->updateGoal(0,1);        // move goal to (0,1)
    dstar->replan();               // plan a path
    mypath = dstar->getPath();     // retrieve path
}

void tick(void)
{
    
}

void cleanup(void)
{
    
}

