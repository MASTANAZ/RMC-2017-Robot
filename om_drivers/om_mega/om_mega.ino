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

#include <Wire.h>
#include <VL53L0X.h>
#include <Servo.h>

////////////////////////////////////////////////////////////////////////////////
// CONSTANTS
////////////////////////////////////////////////////////////////////////////////

// pwm pins which drive the motor controllers
const int MC1 = 3;
const int MC2 = 2;

// limit switch input pins
const int LS_EXCV_EXTENDED = 26;
const int LS_EXCV_RETRACTED = 27;
const int LS_DEPO_UP = 28;
const int LS_DEPO_DOWN = 29;

// the pins that control the states of the automotive relays
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

Servo mc1, mc2;
Servo tf1_servo, tf2_servo;

VL53L0X tf1, tf2;

////////////////////////////////////////////////////////////////////////////////
// FUNCTION DECLARATIONS
////////////////////////////////////////////////////////////////////////////////

void mc1Callback(const std_msgs::Int16& msg);
void mc2Callback(const std_msgs::Int16& msg);
void setControlState(int control_state);

ros::Subscriber<std_msgs::Int16> mc1_sub("mc1", &mc1Callback);
ros::Subscriber<std_msgs::Int16> mc2_sub("mc2", &mc2Callback);

////////////////////////////////////////////////////////////////////////////////
// ARDUINO FUNCTIONS
////////////////////////////////////////////////////////////////////////////////

void setup()
{ 
  node_handle.initNode();

  Wire.begin();
  
  node_handle.subscribe(mc1_sub);
  node_handle.subscribe(mc2_sub);

  pinMode(RELAY_1, OUTPUT);
  pinMode(RELAY_2, OUTPUT);
  pinMode(RELAY_3, OUTPUT);

  pinMode(LS_EXCV_EXTENDED, INPUT);
  pinMode(LS_EXCV_RETRACTED, INPUT);
  pinMode(LS_DEPO_UP, INPUT);
  pinMode(LS_DEPO_DOWN, INPUT);
  
  mc1.attach(MC1);
  mc2.attach(MC2);

  // initialize the TOF sensors
  tf1.init();
  tf1.setMeasurementTimingBudget(200000);
  // TODO: change address of tf2 on the bus to 42
  tf2.init();
  tf2.setMeasurementTimingBudget(200000);
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
  // convert message value of [-100, 100] to [1000, 2000]
  mc1.writeMicroseconds(((int)msg.data * 5) + 1500);
}

void mc2Callback(const std_msgs::Int16& msg)
{
  // convert message value of [-100, 100] to [1000, 2000]
  mc2.writeMicroseconds(((int)msg.data * 5) + 1500);
}

void setControlState(int control_state)
{
  digitalWrite(RELAY_1, RELAY_STATES[control_state][0]);
  digitalWrite(RELAY_2, RELAY_STATES[control_state][1]);
  digitalWrite(RELAY_3, RELAY_STATES[control_state][2]);
}

