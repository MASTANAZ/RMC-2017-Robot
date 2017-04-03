// om_mega.ino
// 
// CREATED BY HARRIS NEWSTEDER AND BLAKE NAZARIO-CASEY
//
// OPERATION CODE FOR THE ON-BOARD ARDUINO MEGA
// NASA RMC 2017

////////////////////////////////////////////////////////////////////////////////
// INCLUDES
////////////////////////////////////////////////////////////////////////////////

#include "ir_sense.h"

#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int8.h>
#include <Servo.h>

////////////////////////////////////////////////////////////////////////////////
// CONSTANTS
////////////////////////////////////////////////////////////////////////////////

// the pins that control the automotive relays
const int RELAY_1 = 8;
const int RELAY_2 = 9;
const int RELAY_3 = 10;

// 
const int CONTROL_STATE_TRVL = 0;
const int CONTROL_STATE_EXCV = 1;
const int CONTROL_STATE_DEPO = 2;

//
const int RELAY_STATES[3][3] = {
  {0, 0, 0},
  {1, 0, 1},
  {0, 1, 0}
};

////////////////////////////////////////////////////////////////////////////////
// NODE VARIABLES
////////////////////////////////////////////////////////////////////////////////

ros::NodeHandle node_handle;

Servo mc1;
Servo mc2;

////////////////////////////////////////////////////////////////////////////////
// FUNCTION DECLARATIONS
////////////////////////////////////////////////////////////////////////////////

void mc1Callback(const std_msgs::Int16& msg);
void mc2Callback(const std_msgs::Int16& msg);
void setControlState(int control_state);

ros::Subscriber<std_msgs::Int16> mc1_sub("lcv", &mc1Callback);
ros::Subscriber<std_msgs::Int16> mc2_sub("rcv", &mc2Callback);

////////////////////////////////////////////////////////////////////////////////
// ARDUINO FUNCTIONS
////////////////////////////////////////////////////////////////////////////////

void setup()
{
  node_handle.initNode();
  node_handle.subscribe(lcv_sub);
  node_handle.subscribe(rcv_sub);

  pinMode(RELAY_1, OUTPUT);
  pinMode(RELAY_2, OUTPUT);
  pinMode(RELAY_3, OUTPUT);
  
  mc1.attach(3);
  mc2.attach(2);
}

void loop()
{
  node_handle.spinOnce();
  delay(5);
}

////////////////////////////////////////////////////////////////////////////////
// FUNCTION DEFINITIONS
////////////////////////////////////////////////////////////////////////////////

void mc1Callback(const std_msgs::Int16& msg)
{
  mc1.writeMicroseconds(((int)msg.data * 5) + 1500);
}

void mc2Callback(const std_msgs::Int16& msg)
{
  mc2.writeMicroseconds(((int)msg.data * 5) + 1500);
}

void setControlState(int control_state)
{
  digitalWrite(RELAY_1, RELAY_STATES[control_state][0]);
  digitalWrite(RELAY_2, RELAY_STATES[control_state][1]);
  digitalWrite(RELAY_3, RELAY_STATES[control_state][2]);
}

