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
#include <math.h>

#include "ros/ros.h"

#include "geometry_msgs/Pose2D.h"

#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Bool.h"

#include "om17/CellCost.h"

////////////////////////////////////////////////////////////////////////////////
// CONSTANTS
////////////////////////////////////////////////////////////////////////////////

const int   STATE_LNCH = 0;
const int   STATE_TTES = 1;
const int   STATE_EXCV = 2;
const int   STATE_TTDS = 3;
const int   STATE_DEPO = 4;

const int   CTRL_STATE_TRVL = 0;
const int   CTRL_STATE_EXCV = 1;
const int   CTRL_STATE_DEPO = 2;

const float OMEGA_ROTATION = 5.0; // Time in seconds to rotate 360 degrees
const float OMEGA_DISTANCE = 0.5; // Time to travel straight 1 meter. Each 
                                  // square is 0.3075 square meters 

const float FIELD_WIDTH = 7.38f;
const float FIELD_HEIGHT = 3.78f;

const int   GRID_WIDTH = 24;
const int   GRID_HEIGHT = GRID_WIDTH / 2;

const float CELL_SIZE = (float)FIELD_WIDTH / (float)GRID_WIDTH;

////////////////////////////////////////////////////////////////////////////////
// NODE VARIABLES
////////////////////////////////////////////////////////////////////////////////

float x = 0.0f, y = 0.0f, theta = 0.0f;

int mc1 = 0, mc2 = 0;

// store measurements for the kalman filter
float tofMeasurements[50] = {};

//
int current_state = -1;

//
bool autonomy_active = true;
bool round_active = false;

// all topics that the auto_controls node subscribes to
ros::Subscriber pose_sub, world_cost_sub;
ros::Subscriber autonomy_active_sub;
ros::Subscriber round_active_sub;

// Get msg data from ToF sensor
ros::Subscriber tof_sensor_sub;

// all topics that the auto_controls node publishes to
ros::Publisher mc1_pub, mc2_pub;
ros::Publisher control_state_pub;
ros::Publisher state_pub;

ros::Time old_time;
ros::Duration tick_elapsed;
float dt = 0.0f;

////////////////////////////////////////////////////////////////////////////////
// FUNCTION DECLARATIONS
////////////////////////////////////////////////////////////////////////////////

void init(ros::NodeHandle &node_handle);
void tick(void);
void cleanup(void);

void setState(int new_state);

void stateTTES(void);
void stateEXCV(void);
void stateTTDS(void);
void stateDEPO(void);

/**
 Callback functions for subscribers
**/
void worldCostCallback(const om17::CellCost::ConstPtr& msg);
void autonomyActiveCallback(const std_msgs::Bool::ConstPtr& msg);
void roundActiveCallback(const std_msgs::Bool::ConstPtr& msg);
void poseCallback(const geometry_msgs::Pose2D::ConstPtr& msg);

#warning THIS WILL NOT BE A STANDARD MESSAGE; ADD CUSTOM MESSAGE TYPE
// Message type required:
//   'ToFSensor'
//
// Parameters:
//   distance (in cm?)
//   angle (radians)  
//
// WILL TOF SENSOR CALLBACK RETURN THE DATA FROM BOTH SENSORS?
void tofSensorCallback(const geometry_msgs::Pose2D::ConstPtr& msg);

/**
 Publisher functions
**/
void publishControls(const ros::TimerEvent& timer_event);

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

    ros::Timer timer = node_handle.createTimer(ros::Duration(0.2), publishControls);

    init(node_handle);
    
    old_time = ros::Time::now();
    
    while (ros::ok())
    {
        tick_elapsed = ros::Time::now() - old_time;
        dt = tick_elapsed.toSec();
        std::cout << dt << std::endl;
        old_time = ros::Time::now();
        
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
    mc1_pub = node_handle.advertise<std_msgs::Int16>("mc1", 10);
    mc2_pub = node_handle.advertise<std_msgs::Int16>("mc2", 10);
    control_state_pub = node_handle.advertise<std_msgs::Int8>("control_state", 10);
    state_pub = node_handle.advertise<std_msgs::Int8>("state", 10);
    
    world_cost_sub = node_handle.subscribe("/world_cost", 10, worldCostCallback);
    round_active_sub = node_handle.subscribe("/round_active", 10, roundActiveCallback);
    pose_sub = node_handle.subscribe("pose", 10, poseCallback);
    autonomy_active_sub = node_handle.subscribe("/autonomy_active", 10, autonomyActiveCallback);
}

void tick(void)
{
    switch (current_state) {
    case STATE_LNCH:
        break;
    case STATE_TTES:
        stateTTES();
        break;
    case STATE_EXCV:
        stateEXCV();
        break;
    case STATE_TTDS:
        stateTTDS();
        break;
    case STATE_DEPO:
        stateDEPO();
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
    
    // let everyone else know
    std_msgs::Int8 out;
    out.data = new_state;
    state_pub.publish(out);

    switch (new_state)
    {

    case STATE_TTES:
        
        break;
    default:
        break;
    }
}

void stateTTES(void)
{
    
}

void stateEXCV(void)
{
    
}

void stateTTDS(void)
{
    
}

void stateDEPO(void)
{
    
}

void worldCostCallback(const om17::CellCost::ConstPtr& msg)
{
    std::cout << msg->x << std::endl;
    std::cout << msg->y << std::endl;
    std::cout << msg->cost << std::endl;
    std::cout << "----------" << std::endl;
}

void autonomyActiveCallback(const std_msgs::Bool::ConstPtr& msg)
{
    autonomy_active = (bool)msg->data;
    // stop all motors
    mc1 = 0;
    mc2 = 0;
}

void roundActiveCallback(const std_msgs::Bool::ConstPtr& msg)
{
    round_active = (bool)msg->data;
    
    // stop all motors
    mc1 = 0;
    mc2 = 0;
    
    if (round_active)
    {
        setState(STATE_TTES);
    }
}

void poseCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    x = (float)msg->x;
    y = (float)msg->y;
    theta = (float)msg->theta;
}

float getAngularTime(float angle_one, float angle_two) {
    return OMEGA_ROTATION * (angle_two - angle_one) / 360.0;
}

float getForwardTime(float pos1[2], float pos2[2]) {
    float distance = sqrt(((pos2[1] - pos1[1])*(pos2[1] - pos1[1])) + ((pos2[0] - pos1[0])*(pos2[0] - pos1[0])));
    return OMEGA_DISTANCE * distance;
}

void publishControls(const ros::TimerEvent& timer_event)
{
    if (!round_active) {
        std_msgs::Int16 send;
        send.data = 0;
        mc1_pub.publish(send);
        send.data = 0;
        mc2_pub.publish(send);
        return;
    }
    
    if (autonomy_active) {
        std_msgs::Int16 send;
        send.data = mc1;
        mc1_pub.publish(send);
        send.data = mc2;
        mc2_pub.publish(send);
    }
}

