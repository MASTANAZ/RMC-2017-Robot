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

#include "excv.h"
#include "ttes.h"
#include "ttds.h"
#include "depo.h"
#include "lnch.h"

#include "robot.h"

#include "DefinedStructs.h"

////////////////////////////////////////////////////////////////////////////////
// CONSTANTS
////////////////////////////////////////////////////////////////////////////////

const int   LOOP_RATE = 30;

const int   STATE_LNCH = 0;
const int   STATE_TTES = 1;
const int   STATE_EXCV = 2;
const int   STATE_TTDS = 3;
const int   STATE_DEPO = 4;

const int   CONTROL_STATE_TRVL = 0;
const int   CONTROL_STATE_EXCV = 1;
const int   CONTROL_STATE_DEPO = 2;

const float OMEGA_ROTATION = 5.0; // Time in seconds to rotate 360 degrees
const float OMEGA_DISTANCE = 0.5; // Time to travel straight 1 meter. Each 
                                  // square is 0.3075 square meters 

const float FIELD_WIDTH = 7.38f;
const float FIELD_HEIGHT = 3.78f;

const int   GRID_WIDTH = 24;
const int   GRID_HEIGHT = GRID_WIDTH / 2;

const float CELL_SIZE = (float)FIELD_WIDTH / (float)GRID_WIDTH;

const float FORWARD_VEL = 0.27f; // Forward velocity in meters per second

////////////////////////////////////////////////////////////////////////////////
// NODE VARIABLES
////////////////////////////////////////////////////////////////////////////////

Robot self;

float targety = 1.89f;

// store measurements for the kalman filter
float tofMeasurements[50] = {};

//
ros::Time old_time;
ros::Duration tick_elapsed;
float dt = 0.0f;
float timer = 0.0f;

//
bool autonomy_active = true;
bool round_active = false;

// subscribers
ros::Subscriber pose_sub, world_cost_sub;
ros::Subscriber autonomy_active_sub;
ros::Subscriber round_active_sub;
ros::Subscriber tof_sensor_sub;

ros::Subscriber ls_excv_ext_sub;
ros::Subscriber ls_excv_ret_sub;
ros::Subscriber ls_depo_all_sub;

// publishers
ros::Publisher mc1_pub, mc2_pub;
ros::Publisher control_state_pub;
ros::Publisher state_pub;

////////////////////////////////////////////////////////////////////////////////
// FUNCTION DECLARATIONS
////////////////////////////////////////////////////////////////////////////////

//
void init(ros::NodeHandle &node_handle);
void tick(void);
void cleanup(void);

//
void setState(int new_state, bool external_call = false);
void setControlState(int new_control_state);

//
void stateTTES(void);
void stateEXCV(void);
void stateTTDS(void);
void stateDEPO(void);
void stateLNCH(void);

// subscribers callbacks
void worldCostCallback(const om17::CellCost::ConstPtr& msg);
void autonomyActiveCallback(const std_msgs::Bool::ConstPtr& msg);
void roundActiveCallback(const std_msgs::Bool::ConstPtr& msg);
void poseCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
void stateCallback(const std_msgs::Int8::ConstPtr& msg);
void lsExcvExtCallback(const std_msgs::Bool::ConstPtr& msg);
void lsExcvRetCallback(const std_msgs::Bool::ConstPtr& msg);
void lsDepoAllCallback(const std_msgs::Bool::ConstPtr& msg);

// 
void publishControls(const ros::TimerEvent& timer_event);

//
void calibrateRotation();

////////////////////////////////////////////////////////////////////////////////
// ENTRY POINT
////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
    // ROS initialization
    ros::init(argc, argv, "auto_controls");
    ros::NodeHandle node_handle;
    ros::Rate loop_rate(LOOP_RATE);

    ros::Timer timer = node_handle.createTimer(ros::Duration(0.2), publishControls);

    init(node_handle);
    
    // makes sure the initial dt value is zero
    old_time = ros::Time::now();
    
    while (ros::ok())
    {
        // delta time calculation used for autonomy
        tick_elapsed = ros::Time::now() - old_time;
        dt = tick_elapsed.toSec();
        old_time = ros::Time::now();
        
        //
        if (round_active)
        {
            // autonomous control
            if (autonomy_active)
            {
                tick();
            }
            // manual control
            else
            {
                // do nothing
            }
        }
        // round is not active, robot should not be moving
        else
        {
            self.mc1 = 0;
            self.mc2 = 0;
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
    // publishers
    mc1_pub = node_handle.advertise<std_msgs::Int16>("mc1", 10);
    mc2_pub = node_handle.advertise<std_msgs::Int16>("mc2", 10);
    control_state_pub = node_handle.advertise<std_msgs::Int8>("control_state", 10);
    state_pub = node_handle.advertise<std_msgs::Int8>("state", 10);
    
    // subscribers
    world_cost_sub = node_handle.subscribe("/world_cost", 10, worldCostCallback);
    round_active_sub = node_handle.subscribe("/round_active", 10, roundActiveCallback);
    pose_sub = node_handle.subscribe("pose", 10, poseCallback);
    autonomy_active_sub = node_handle.subscribe("/autonomy_active", 10, autonomyActiveCallback);
    
    ls_excv_ext_sub = node_handle.subscribe("ls_excv_ext", 10, lsExcvExtCallback);
    ls_excv_ret_sub = node_handle.subscribe("ls_excv_ret", 10, lsExcvRetCallback);
    ls_depo_all_sub = node_handle.subscribe("ls_depo_all", 10, lsDepoAllCallback);
}

void tick(void)
{
    switch (self.current_state) {
    case STATE_LNCH:
        stateLNCH();
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
    
}

void setState(int new_state, bool external_call)
{
    self.current_state = new_state;

    // this function was called from outside the program and we don't need to
    // publish the new state value (this would result in an infinite loop)
    if (!external_call)
    {
      std_msgs::Int8 msg;
      msg.data = new_state;
      state_pub.publish(msg);
    }
    
    // stop all motors on a state switch
    self.mc1 = 0;
    self.mc2 = 0;
    
    // force publish controls
    std_msgs::Int16 send;
    send.data = self.mc1;
    mc1_pub.publish(send);
    send.data = self.mc2;
    mc2_pub.publish(send);

    switch (new_state)
    {
    case STATE_LNCH:
        setControlState(CONTROL_STATE_TRVL);
        lnch::init(&self);
        break;
    case STATE_TTES:
        setControlState(CONTROL_STATE_TRVL);
        ttes::init(&self);
        break;
    case STATE_TTDS:
        setControlState(CONTROL_STATE_TRVL);
        ttds::init(&self);
        break;
    case STATE_EXCV:
        setControlState(CONTROL_STATE_EXCV);
        excv::init(&self);
        break;
    case STATE_DEPO:
        setControlState(CONTROL_STATE_DEPO);
        depo::init(&self);
        break;
    default:
        break;
    }
}

void setControlState(int new_control_state)
{
    std_msgs::Int8 msg;
    self.control_state = new_control_state;
    msg.data = new_control_state;
    control_state_pub.publish(msg);
}

////////////////////////////////////////////////////////////////////////////////
// STATE FUNCTIONS
////////////////////////////////////////////////////////////////////////////////

void stateLNCH(void)
{
    lnch::tick(dt,&self);
    
    if (lnch::changeState())
    {
        setState(STATE_TTES);
    }
}


void stateTTES(void)
{
    // follow d-star path to excavation area
    //setState(STATE_EXCV);
    ttes::tick(dt, &self);
    
    if (ttes::changeState())
    {
        setState(STATE_EXCV);
    }
}

void stateEXCV(void)
{
    excv::tick(dt, &self);
    
    if (excv::changeState())
    {
        setState(STATE_TTDS);
    }
}

void stateTTDS(void)
{
    // follow d-star path deposition area
    ttds::tick(dt, &self);
    
    if (ttds::changeState())
    {
        setState(STATE_DEPO);
    }
}

void stateDEPO(void)
{
    // arduino handles the deposition cycle
     // arduino handles the deposition cycle
    depo::tick(dt, &self);
    
    if (depo::changeState())
    {
        setState(STATE_EXCV);
    }
}

////////////////////////////////////////////////////////////////////////////////
// SUBSCRIBER CALLBACKS
////////////////////////////////////////////////////////////////////////////////

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
}

void roundActiveCallback(const std_msgs::Bool::ConstPtr& msg)
{
    round_active = (bool)msg->data;
    
    if (round_active)
    {
        setState(STATE_DEPO);
    }
}

void poseCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    self.x = (float)msg->x;
    self.y = (float)msg->y;
    self.theta = (float)msg->theta;
}

void lsExcvExtCallback(const std_msgs::Bool::ConstPtr& msg)
{
    self.ls_excv_ext = msg->data;
}

void lsExcvRetCallback(const std_msgs::Bool::ConstPtr& msg)
{
    self.ls_excv_ret = msg->data;
}

void lsDepoAllCallback(const std_msgs::Bool::ConstPtr& msg)
{
    self.ls_depo_all = msg->data;
}

////////////////////////////////////////////////////////////////////////////////
// 
////////////////////////////////////////////////////////////////////////////////

void publishControls(const ros::TimerEvent& timer_event)
{
    if (round_active && !autonomy_active) return;
    
    std_msgs::Int16 send;
    send.data = self.mc1;
    mc1_pub.publish(send);
    send.data = self.mc2;
    mc2_pub.publish(send);
}


