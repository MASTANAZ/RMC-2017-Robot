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

#include "dstar.h"

#include "follower.h"

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

const float CELL_SIZE = (float)FIELD_WIDTH / (float)GRID_WIDTH;

////////////////////////////////////////////////////////////////////////////////
// NODE VARIABLES
////////////////////////////////////////////////////////////////////////////////

int mc1, mc2;

Point position;
float theta = 0;

// store measurements for the kalman filter
float tofMeasurements[50] = {};

// 
int current_state = -1;

//
bool autonomy_active = true;
bool round_active = false;

// path planning variables
Dstar *dstar = nullptr;
list<state> mypath;

// all topics that the auto_controls node subscribes to
ros::Subscriber pose_sub, world_cost_sub;
ros::Subscriber autonomy_active_sub;
ros::Subscriber round_active_sub;

// Get msg data from ToF sensor
ros::Subscriber tof_sensor_sub;

// all topics that the auto_controls node publishes to
ros::Publisher mc1_pub, mc2_pub, control_state_pub, state_pub;

Follower follower;

////////////////////////////////////////////////////////////////////////////////
// FUNCTION DECLARATIONS
////////////////////////////////////////////////////////////////////////////////

void init(ros::NodeHandle &node_handle);
void tick(void);
void cleanup(void);

void setState(int new_state);

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
    mc1_pub = node_handle.advertise<std_msgs::Int16>("mc1", 10);
    mc2_pub = node_handle.advertise<std_msgs::Int16>("mc2", 10);
    control_state_pub = node_handle.advertise<std_msgs::Int8>("control_state", 10);
    state_pub = node_handle.advertise<std_msgs::Int8>("state", 10);
    
    

    world_cost_sub = node_handle.subscribe("/world_cost", 10, worldCostCallback);
    round_active_sub = node_handle.subscribe("/round_active", 10, roundActiveCallback);
    pose_sub = node_handle.subscribe("pose", 10, poseCallback);
    autonomy_active_sub = node_handle.subscribe("/autonomy_active", 10, autonomyActiveCallback);

    // Subscribe to ToF sensor topic
    //tof_sensor_sub = node_handle.subscribe("tof_sensor", 10, tofSensorCallback);
    
    dstar = new Dstar();
}

void tick(void)
{
    // Increase the tick count by 1 every tick.
    // The tick is at 30/second. 
    switch (current_state) {
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
    
    std::vector<Point> tmp;


    switch (new_state)
    {

    case STATE_TTES:
        std::cout << (int)(position.x / CELL_SIZE) << ", " << (int)(position.y / CELL_SIZE) << std::endl;
        dstar->init((int)(position.x / CELL_SIZE),(int)(position.y / CELL_SIZE), 18, 6);
        dstar->replan();
        mypath = dstar->getPath();
        
        
        for (state s : mypath)
        {
            Point t;
            t.x = s.x;
            t.y = s.y;
            std::cout << t.x << ", ";
            std::cout << t.y << std::endl;
            
            tmp.push_back(t);
        }

        follower.processPath(tmp);

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
    position.x = (float)msg->x;
    position.y = (float)msg->y;
    
    // Move the thetas down the queue
    theta = (float)msg->theta;
}

/**
  msg->x     = Left sensor distance
  msg->y     = Right sensor distance
  msg->theta = Angle of each sensor (they share the same answer)
**/

/*
void tofSensorCallback(const geometry_msgs::Pose2D::ConstPtr& msg) {
  // ToF sensor stuff
  float leftDist   = (float)msg->x;
  float rightDist  = (float)msg->y;
  float theta = (float)msg->theta;
  
  float distAvg = (leftDist + rightDist) / 2.0f;
  
  float dx = x + (distAvg * cos(theta));
  float dy = y + (distAvg * sin(theta));
  
  float gridX = floor(dx / CELL_SIZE);
  float gridY = floor(dy / CELL_SIZE);
} */

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

/*/ Determine if the rover needs to turn left/right or forward/backward
void getLCVandRCVvalues(void) {
    float delta_x, delta_y, delta_r = 0.0;
    float delta_len = 0.0f;

    delta_x = real_path.at(next_index).x - x;
    delta_y = real_path.at(next_index).y - y;
    
    delta_len = (float)sqrt(pow(delta_x, 2) + pow(delta_y, 2));
    
    std::cout << "dl = "<< delta_len << std::endl;
    
    //if (delta_len < 0.01f) next_index++;
    
    std::cout << "dx = " << delta_x << std::endl;
    std::cout << "dy = " << delta_y << std::endl;

    float phi = (float)atan(delta_y / delta_x);
    
    std::cout << "theta = " << theta << std::endl;
    std::cout << "phi = " << phi << std::endl;
    
    delta_r = phi - theta;
    
    std::cout << "dr = " << delta_r << std::endl;
    
    if (delta_r > 0.2) {
        mc1_drive = -1;
        rcv_drive = 1;
    }
    else if (delta_r <= 0.2 && delta_r >= -0.2) {
        mc1_drive = 1;
        rcv_drive = 1;
    }
    else if (delta_r < -0.2) {
        mc1_drive = 1;
        rcv_drive = -1;
    }
    
    std::cout << "mc1 : " << mc1_drive << std::endl;
    std::cout << "rcv : " << rcv_drive << std::endl;
}*/



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

