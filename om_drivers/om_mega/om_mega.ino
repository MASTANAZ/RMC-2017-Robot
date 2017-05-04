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
#include <Servo.h>

////////////////////////////////////////////////////////////////////////////////
// CONSTANTS
////////////////////////////////////////////////////////////////////////////////

// pwm pins which drive the motor controllers
const int MC1 = 3;
const int MC2 = 2;

// limit switch input pins
const int LS_EXCV_EXTENDED = 26;
const int LS_EXCV_RETRACTED = 29;
const int LS_DEPO_UP = 28;
const int LS_DEPO_DOWN = 27;

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
Servo tf1_servo, tf2_servo;

//VL53L0X tf1, tf2;

int control_state = 0;

// set to true to force stop movement of all motors;
// useful when limit switches are activated
bool force_stop = false;

long old_time, new_time;
float dt;

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
  
  node_handle.subscribe(mc1_sub);
  node_handle.subscribe(mc2_sub);
  node_handle.subscribe(control_state_sub);

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
  //tf1.init();
  //tf1.setMeasurementTimingBudget(200000);
  // TODO: change address of tf2 on the bus to 42 (NEED TO USE XSHUT PIN)
  //tf2.init();
  //tf2.setMeasurementTimingBudget(200000);

  // ensures the first value of dt is zero
  old_time = millis();
}

void loop()
{
  delay(5);
  
  new_time = millis();
  dt = (float)(new_time - old_time) / 1000.0f;
  old_time = new_time;
  
  node_handle.spinOnce();

  if (control_state == CONTROL_STATE_TRVL)
  {
    // auto controls controls everything here; do nothing
    force_stop = false;
  }
  else if (control_state == CONTROL_STATE_EXCV) 
  {
    // excavation is happening, check limit switches to make sure the linear
    // position of the excavator isn't out of bounds
    if (digitalRead(LS_EXCV_RETRACTED) == HIGH)
    {
      force_stop = true;
      mc1.writeMicroseconds(1500);
    }
    else
    {
      force_stop = false; 
    }
  }
  else if (control_state == CONTROL_STATE_DEPO)
  {
    force_stop = false;
    // deposition cycle, check limit switches to make sure position of the
    // deposition bucket isn't out of bounds
  }
}

////////////////////////////////////////////////////////////////////////////////
// FUNCTION DEFINITIONS
////////////////////////////////////////////////////////////////////////////////

// Motor Controller 1 callback
void mc1Callback(const std_msgs::Int16& msg)
{
  if (force_stop) return;
  
  mc1_val = msg.data;
  // convert message value of [-100, 100] to [1000, 2000]
  mc1.writeMicroseconds(((int)msg.data * 5) + 1500);
}

void mc2Callback(const std_msgs::Int16& msg)
{
  // don't set new values while the motors are in force stop
  if (force_stop) return;
  
  mc2_val = msg.data;
  // convert message value of [-100, 100] to [1000, 2000]
  mc2.writeMicroseconds(((int)msg.data * 5) + 1500);
}

void controlStateCallback(const std_msgs::Int8& msg)
{
  control_state = msg.data;
  
  // Control State Callback data
  digitalWrite(RELAY_1, RELAY_STATES[msg.data][0]);
  digitalWrite(RELAY_2, RELAY_STATES[msg.data][1]);
  digitalWrite(RELAY_3, RELAY_STATES[msg.data][2]);
}

