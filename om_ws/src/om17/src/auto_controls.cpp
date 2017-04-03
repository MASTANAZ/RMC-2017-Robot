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

const float FIELD_WIDTH = 7.38f;
const float FIELD_HEIGHT = 3.78f;

const int GRID_WIDTH = 24;
const int GRID_HEIGHT = GRID_WIDTH / 2;

const float CELL_SIZE = FIELD_WIDTH / (float)GRID_WIDTH;

////////////////////////////////////////////////////////////////////////////////
// NODE VARIABLES
////////////////////////////////////////////////////////////////////////////////

float x[2] = {0.0f, 0.0f}, y[2] = {0.0f, 0.0f};

// Array of previous angle and current angle
float thetas[2] = {0.0, 0.0};

unsigned int tick_count = 0;

int lcv = 0, rcv = 0;
int lcv_rcv[2] = {0,0};

// 
int current_state = -1;

//
bool autonomy_active = true;
bool round_active = false;

// path planning variables
Dstar *dstar = nullptr;
list<state> mypath;
//list<state> path;

// all topics that the auto_controls node subscribes to
ros::Subscriber pose_sub, world_cost_sub;
ros::Subscriber autonomy_active_sub;
ros::Subscriber round_active_sub;

// all topics that the auto_controls node publishes to
ros::Publisher lcv_pub, rcv_pub, control_state_pub, state_pub;

////////////////////////////////////////////////////////////////////////////////
// FUNCTION DECLARATIONS
////////////////////////////////////////////////////////////////////////////////

void init(ros::NodeHandle &node_handle);
void tick(void);
void cleanup(void);

void setState(int new_state);

void worldCostCallback(const om17::CellCost::ConstPtr& msg);
void autonomyActiveCallback(const std_msgs::Bool::ConstPtr& msg);
void roundActiveCallback(const std_msgs::Bool::ConstPtr& msg);
void poseCallback(const geometry_msgs::Pose2D::ConstPtr& msg);

void publishControls(const ros::TimerEvent& timer_event);

void pathTest();
float getAngularTime(float angle_one, float angle_two); 
float getForwardTime(float pos1[2], float pos2[2]);
void getLCVandRCVvalues(float pos1[2], float pos2[2], float angles[2]);

////////////////////////////////////////////////////////////////////////////////
// ENTRY POINT
////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
    // ROS initialization
    ros::init(argc, argv, "auto_controls");
    ros::NodeHandle node_handle;
    ros::Rate loop_rate(30);

    ros::Timer timer = node_handle.createTimer(ros::Duration(0.2), publishControls);

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
    lcv_pub = node_handle.advertise<std_msgs::Int16>("mc1", 10);
    rcv_pub = node_handle.advertise<std_msgs::Int16>("mc2", 10);
    control_state_pub = node_handle.advertise<std_msgs::Int8>("control_state", 10);
    state_pub = node_handle.advertise<std_msgs::Int8>("state", 10);

    world_cost_sub = node_handle.subscribe("/world_cost", 10, worldCostCallback);
    round_active_sub = node_handle.subscribe("/round_active", 10, roundActiveCallback);
    pose_sub = node_handle.subscribe("pose", 10, poseCallback);
    autonomy_active_sub = node_handle.subscribe("/autonomy_active", 10, autonomyActiveCallback);

    dstar = new Dstar();
}

void tick(void)
{
    // Increase the tick count by 1 every tick.
    // The tick is at 30/second. 
    tick_count++;
    switch (current_state) {
    case STATE_LNCH:
        break;
    case STATE_TTES:
        getLCVandRCVvalues(x, y, thetas);
        
       
/*
        if (x < 4.4) {
            lcv = 80;
            rcv = 80;
        } else {
            lcv = 0;
            rcv = 0;
            setState(STATE_EXCV);
        }

**/
        break;
    case STATE_EXCV:
        break;
    case STATE_TTDS:
        break;
    case STATE_DEPO:
        break;
    default:
        break;
    } 
}

void cleanup(void)
{
    delete dstar;
}

void setState(int new_state)
{
    current_state = new_state;
    std_msgs::Int8 send;
    send.data = new_state;
    state_pub.publish(send);
    
    switch (new_state)
    {
    case STATE_TTES:
        std::cout << (int)(x[1] / CELL_SIZE) << ", " << (int)(y[1] / CELL_SIZE) << std::endl;
    
        dstar->init((int)(x[1] / CELL_SIZE),(int)(y[1] / CELL_SIZE), 18, 6);
        mypath = dstar->getPath();
        break;
    default:
        break;
    }
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
    lcv = 0;
    rcv = 0;
}

void roundActiveCallback(const std_msgs::Bool::ConstPtr& msg)
{
    round_active = (bool)msg->data;
    // stop all motors
    lcv = 0;
    rcv = 0;
    
    if (round_active)
    {
        setState(STATE_TTES);
    }
}

void poseCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    x[0]      = x[1];
    x[1]      = (float)msg->x;

    y[0]      = y[1];
    y[1]      = (float)msg->y;
    
    // Move the thetas down the queue
    thetas[0] = thetas[1];
    thetas[1] = (float)msg->theta;

    dstar->updateStart((int)(x[1] / CELL_SIZE),(int)(y[1] / CELL_SIZE));
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

// Determine if the rover needs to turn left/right or forward/backward
void getLCVandRCVvalues(float pos1[2], float pos2[2], float angles[2]) {
    float delta_x, delta_y, delta_r = 0.0;

    delta_x = pos2[0] - pos1[0];
    delta_y = pos2[1] - pos1[1];
    delta_r = angles[1] - angles[0];

    int lcv_rcv[2] = {0,0};
    
    if (delta_x > 0 && delta_y > 0 && delta_r > 0) {
        lcv_rcv[0] = -1;
        lcv_rcv[1] = 1;
    }

    else if (delta_x > 0 && delta_y > 0 && delta_r == 0) {
        lcv_rcv[0] = 1;
        lcv_rcv[1] = 1;
    }

    else if (delta_x > 0 && delta_y < 0 && delta_r < 0) {
        lcv_rcv[0] = 1;
        lcv_rcv[1] = -1;
    }

    else if (delta_x < 0 && delta_y < 0 && delta_r < 0) {
        lcv_rcv[0] = -1;
        lcv_rcv[1] = -1;
    }

    else if (delta_x < 0 && delta_y < 0 && delta_r == 0) {
        lcv_rcv[0] = -1;
        lcv_rcv[1] = -1;
    }

}

void publishControls(const ros::TimerEvent& timer_event)
{
    if (!round_active) {
        std_msgs::Int16 send;
        send.data = 0;
        lcv_pub.publish(send);
        send.data = 0;
        rcv_pub.publish(send);
        return;
    }
    
    if (autonomy_active) {
        std_msgs::Int16 send;
        send.data = lcv;
        lcv_pub.publish(send);
        send.data = rcv;
        rcv_pub.publish(send);
    }
}

