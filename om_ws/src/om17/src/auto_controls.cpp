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
#include "std_msgs/Bool.h"

#include "om17/CellCost.h"

#include "dstar.h"

////////////////////////////////////////////////////////////////////////////////
// CONSTANTS
////////////////////////////////////////////////////////////////////////////////

const int STATE_LNCH = 0;
const int STATE_TTES = 1;
const int STATE_EXCV = 2;
const int STATE_TTDS = 3;
const int STATE_DEPO = 4;

const int CTRL_STATE_TRVL = 0;
const int CTRL_STATE_EXCV = 1;
const int CTRL_STATE_DEPO = 2;

const float OMEGA_ROTATION = 5.0; // Time in seconds to rotate 360 degrees
const float OMEGA_DISTANCE = 0.5; // Time to travel straight 1 meter. Each 
                                  // square is 0.3075 square meters 

////////////////////////////////////////////////////////////////////////////////
// NODE VARIABLES
////////////////////////////////////////////////////////////////////////////////

int current_state = STATE_LNCH;

bool autonomy_active = true;
bool round_active = false;

// path planning variables
Dstar *dstar = nullptr;
list<state> path;

// all topics that the auto_controls node subscribes to
ros::Subscriber pose_sub, world_cost_sub;
ros::Subscriber autonomy_active_sub;
ros::Subscriber round_active_sub;

// all topics that the auto_controls node publishes to
ros::Publisher lcv_pub, rcv_pub, control_state_pub;

////////////////////////////////////////////////////////////////////////////////
// FUNCTION DECLARATIONS
////////////////////////////////////////////////////////////////////////////////

void init(ros::NodeHandle &node_handle);
void tick(void);
void cleanup(void);
void worldCostCallback(const om17::CellCost::ConstPtr& msg);
void autonomyActiveCallback(const std_msgs::Bool::ConstPtr& msg);
void roundActiveCallback(const std_msgs::Bool::ConstPtr& msg);
void poseCallback (const geometry_msgs::Pose2D::ConstPtr& msg);

void pathTest();
float getAngularTime(float angle_one, float angle_two); 
float getForwardTime(float pos1[2], float pos2[2]);

////////////////////////////////////////////////////////////////////////////////
// ENTRY POINT
////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
    // ROS initialization
    ros::init(argc, argv, "auto_controls");
    ros::NodeHandle node_handle;
    ros::Rate loop_rate(30);

    init(node_handle);
    
    while (ros::ok())
    {
        if (autonomy_active && round_active)
        {
            tick();
        }

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
    lcv_pub = node_handle.advertise<std_msgs::Int16>("lcv", 10);
    rcv_pub = node_handle.advertise<std_msgs::Int16>("rcv", 10);
    control_state_pub = node_handle.advertise<std_msgs::Int8>("control_state", 10);

    // the global world cost
    world_cost_sub = node_handle.subscribe("/world_cost", 10, worldCostCallback);
    round_active_sub = node_handle.subscribe("/round_active", 10, roundActiveCallback);
    pose_sub = node_handle.subscribe("pose", 10, poseCallback);
    autonomy_active_sub = node_handle.subscribe("/autonomy_active", 10, autonomyActiveCallback);

    dstar = new Dstar();
}

void worldCostCallback(const om17::CellCost::ConstPtr& msg)
{
    std::cout << msg->x << std::endl;
    std::cout << msg->y << std::endl;
    std::cout << msg->cost << std::endl;
    std::cout << "----------" << std::endl;
    //dstar->updateCell(msg->x, msg->y, msg->cost);
    //dstar->replan();
}

void autonomyActiveCallback(const std_msgs::Bool::ConstPtr& msg)
{
    autonomy_active = (bool)msg->data;
    // stop all motors
}

void roundActiveCallback(const std_msgs::Bool::ConstPtr& msg)
{
    round_active = (bool)msg->data;
    // stop all motors;
}

void poseCallback (const geometry_msgs::Pose2D::ConstPtr& msg)
{
    float x     = (float)msg->x;
    float y     = (float)msg->y;
    float theta = (float)msg->theta;
}

void pathTest()
{
    /*Dstar *dstar = new Dstar();
    list<state> mypath;

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
    mypath = dstar->getPath();     // retrieve path*/
}

float getAngularTime(float angle_one, float angle_two) {
    return OMEGA_ROTATION * (angle_two - angle_one) / 360.0;
}

float getForwardTime(float pos1[2], float pos2[2]) {
    float distance = sqrt(((pos2[1] - pos1[1])*(pos2[1] - pos1[1])) + ((pos2[0] - pos1[0])*(pos2[0] - pos1[0])));
    return OMEGA_DISTANCE * distance;
}

void tick(void)
{
    /*switch (current_state) {
    case STATE_LNCH:
        break;
    case STATE_TTES:
        break;
    case STATE_EXCV:
        break;
    case STATE_TTDS:
        break;
    case STATE_DEPO:
        break;
    }*/

    
}

void cleanup(void)
{
    delete dstar;
}

