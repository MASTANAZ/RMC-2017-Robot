// om_mega.ino
// 
// CREATED BY HARRIS NEWSTEDER AND BLAKE NAZARIO-CASEY
//
// OPERATION CODE FOR THE ON-BOARD ARDUINO MEGA
// NASA RMC 2017

////////////////////////////////////////////////////////////////////////////////
// INCLUDES
////////////////////////////////////////////////////////////////////////////////

#include <ros.h>

#include <std_msgs/Int16.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>

#include <Wire.h>
#include <Servo.h>

#include "tof.h"

////////////////////////////////////////////////////////////////////////////////
// CONSTANTS
////////////////////////////////////////////////////////////////////////////////

// pwm pins which drive the motor controllers
const int MC1 = 3;
const int MC2 = 2;

// limit switch input pins
const int LS_EXCV_EXTENDED = 29;
const int LS_EXCV_RETRACTED = 28;
const int LS_DEPO = 27;

// the pins that control the states of the automotive relays
const int RELAY_1 = 22;
const int RELAY_2 = 23;
const int RELAY_3 = 24;

// 
const int CONTROL_STATE_TRVL = 0;
const int CONTROL_STATE_EXCV = 1;
const int CONTROL_STATE_DEPO = 2;

//
const int RELAY_STATES[3][3] = {
  {0, 0, 0}, // CONTROL_STATE_TRVL
  {1, 0, 1}, // CONTROL_STATE_EXCV
  {0, 1, 0}  // CONTROL_STATE_DEPO
};

////////////////////////////////////////////////////////////////////////////////
// FUNCTION DECLARATIONS
////////////////////////////////////////////////////////////////////////////////

void mc1Callback(const std_msgs::Int16& msg);
void mc2Callback(const std_msgs::Int16& msg);
void controlStateCallback(const std_msgs::Int8& msg);

////////////////////////////////////////////////////////////////////////////////
// NODE VARIABLES
////////////////////////////////////////////////////////////////////////////////

ros::NodeHandle node_handle;

int mc1_val = 0;
int mc2_val = 0;

Servo mc1, mc2;

int control_state = 0;

long old_time, new_time;
float dt;

std_msgs::Bool ls_excv_ext_msg;
std_msgs::Bool ls_excv_ret_msg;
std_msgs::Bool ls_depo_all_msg;

ros::Publisher ls_excv_ext_pub("ls_excv_ext", &ls_excv_ext_msg);
ros::Publisher ls_excv_ret_pub("ls_excv_ret", &ls_excv_ret_msg);
ros::Publisher ls_depo_all_pub("ls_depo_all", &ls_depo_all_msg);

ros::Subscriber<std_msgs::Int16> mc1_sub("mc1", &mc1Callback);
ros::Subscriber<std_msgs::Int16> mc2_sub("mc2", &mc2Callback);
ros::Subscriber<std_msgs::Int8> control_state_sub("control_state", &controlStateCallback);

////////////////////////////////////////////////////////////////////////////////
// ARDUINO FUNCTIONS
////////////////////////////////////////////////////////////////////////////////

void setup()
{
  node_handle.initNode();
  
  // initialilze i2c bus communications
  Wire.begin();
  
  ls_excv_ext_msg.data = false;
  ls_excv_ret_msg.data = false;
  ls_depo_all_msg.data = false;
  
  node_handle.advertise(ls_excv_ext_pub);
  node_handle.advertise(ls_excv_ret_pub);
  node_handle.advertise(ls_depo_all_pub);
  
  node_handle.subscribe(mc1_sub);
  node_handle.subscribe(mc2_sub);
  node_handle.subscribe(control_state_sub);

  pinMode(RELAY_1, OUTPUT);
  pinMode(RELAY_2, OUTPUT);
  pinMode(RELAY_3, OUTPUT);

  pinMode(LS_EXCV_EXTENDED, INPUT);
  pinMode(LS_EXCV_RETRACTED, INPUT);
  pinMode(LS_DEPO, INPUT);
  
  mc1.attach(MC1);
  mc2.attach(MC2);
  
  // initialize TOF sensors
  tof::init();

  // ensures the first value of dt is zero
  old_time = millis();
}

void loop()
{
  delay(1);

  // delta time calculation
  new_time = millis();
  dt = (float)(new_time - old_time) / 1000.0f;
  old_time = new_time;
  
  node_handle.spinOnce();
  
  //tof::tick(dt);
  
  // excavator extension limit switch
  if (ls_excv_ext_msg.data)
  {
    if (digitalRead(LS_EXCV_EXTENDED) == LOW)
    {
      ls_excv_ext_msg.data = false;
      ls_excv_ext_pub.publish(&ls_excv_ext_msg);
    }
  }
  else
  {
    if (digitalRead(LS_EXCV_EXTENDED) == HIGH)
    {
      ls_excv_ext_msg.data = true;
      ls_excv_ext_pub.publish(&ls_excv_ext_msg);
    }
  }
  
  // excavator retraction limit switch
  if (ls_excv_ret_msg.data)
  {
    if (digitalRead(LS_EXCV_RETRACTED) == LOW)
    {
      ls_excv_ret_msg.data = false;
      ls_excv_ret_pub.publish(&ls_excv_ret_msg);
    }
  }
  else
  {
    if (digitalRead(LS_EXCV_RETRACTED) == HIGH)
    {
      ls_excv_ret_msg.data = true;
      ls_excv_ret_pub.publish(&ls_excv_ret_msg);
    }
  }
  
  // deposition limit switch
  if (ls_depo_all_msg.data)
  {
    if (digitalRead(LS_DEPO) == LOW)
    {
      ls_depo_all_msg.data = false;
      ls_depo_all_pub.publish(&ls_depo_all_msg);
    }
  }
  else
  {
    if (digitalRead(LS_DEPO) == HIGH)
    {
      ls_depo_all_msg.data = true;
      ls_depo_all_pub.publish(&ls_depo_all_msg);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
// FUNCTION DEFINITIONS
////////////////////////////////////////////////////////////////////////////////

void mc1Callback(const std_msgs::Int16& msg)
{
  // convert message value of [-100, 100] to [1000, 2000]
  mc1.writeMicroseconds(((int)msg.data * 5) + 1500);
}

void mc2Callback(const std_msgs::Int16& msg)
{
  // convert message value of [-100, 100] to [1000, 2000]
  mc2.writeMicroseconds(((int)msg.data * 5) + 1500);
}

void controlStateCallback(const std_msgs::Int8& msg)
{
  control_state = msg.data;
  
  digitalWrite(RELAY_1, RELAY_STATES[msg.data][0]);
  digitalWrite(RELAY_2, RELAY_STATES[msg.data][1]);
  digitalWrite(RELAY_3, RELAY_STATES[msg.data][2]);
}

