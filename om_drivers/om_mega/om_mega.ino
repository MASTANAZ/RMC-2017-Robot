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

ros::NodeHandle nh;

Servo lcv;
Servo rcv;

////////////////////////////////////////////////////////////////////////////////
// FUNCTION DECLARATIONS
////////////////////////////////////////////////////////////////////////////////

void lcvCallback(const std_msgs::Int16& lcv_msg);
void rcvCallback(const std_msgs::Int16& rcv_msg);
void setControlState(int control_state);

ros::Subscriber<std_msgs::Int16> lcv_sub("lcv", &lcvCallback);
ros::Subscriber<std_msgs::Int16> rcv_sub("rcv", &rcvCallback);

////////////////////////////////////////////////////////////////////////////////
// ARDUINO FUNCTIONS
////////////////////////////////////////////////////////////////////////////////

void setup()
{
  nh.initNode();
  nh.subscribe(lcv_sub);
  nh.subscribe(rcv_sub);

  pinMode(RELAY_1, OUTPUT);
  pinMode(RELAY_2, OUTPUT);
  pinMode(RELAY_3, OUTPUT);
  
  lcv.attach(3);
  rcv.attach(2);
}

void loop()
{
  nh.spinOnce();
  delay(5);
}

////////////////////////////////////////////////////////////////////////////////
// FUNCTION DEFINITIONS
////////////////////////////////////////////////////////////////////////////////

void lcvCallback(const std_msgs::Int16& lcv_msg)
{
  lcv.writeMicroseconds(((int)lcv_msg.data * 5) + 1500);
}

void rcvCallback(const std_msgs::Int16& rcv_msg)
{
  lcv.writeMicroseconds(((int)rcv_msg.data * 5) + 1500);
}

void setControlState(int control_state)
{
  digitalWrite(RELAY_1, RELAY_STATES[control_state][0]);
  digitalWrite(RELAY_2, RELAY_STATES[control_state][1]);
  digitalWrite(RELAY_3, RELAY_STATES[control_state][2]);
}

