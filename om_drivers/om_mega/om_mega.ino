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

const int DEG_PER_SECOND = 80;

const int TOF_ANGLE_MIN = 80;
const int TOF_ANGLE_MAX = 140;

//
const int TOF_PWM = 4;

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
Servo tof;

//VL53L0X tf1, tf2;

int control_state = 0;

// 
bool mc1_fs_p = false;
bool mc1_fs_n = false;

bool mc2_fs_p = false;
bool mc2_fs_n = false;

float rot_timer = 0.0f;
int tof_angle = 0;
int inc = 1;

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
  
  //tof.attach(TOF_PWM);
  //tof.writeMicroseconds((int)((tof_angle / 180.0f) * 1000.0f) + 1000);
  
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

  // delta time calculation
  new_time = millis();
  dt = (float)(new_time - old_time) / 1000.0f;
  old_time = new_time;
  
  node_handle.spinOnce();

  if (control_state == CONTROL_STATE_TRVL)
  {
    // auto controls controls everything here; do nothing
  }
  else if (control_state == CONTROL_STATE_EXCV) 
  {
    // excavation is happening, check limit switches to make sure the linear
    // position of the excavator isn't out of bounds
    if (digitalRead(LS_EXCV_RETRACTED) == HIGH)
    {
      mc1_fs_p = true;
      if (mc1_val > 0)
      {
        mc1_val = 0;
        mc1.writeMicroseconds(1500);
      }
    }
    else
    {
      mc1_fs_p = false; 
    }
  }
  else if (control_state == CONTROL_STATE_DEPO)
  {
    // deposition cycle, check limit switches to make sure position of the
    // deposition bucket isn't out of bounds
  }
}

////////////////////////////////////////////////////////////////////////////////
// FUNCTION DEFINITIONS
////////////////////////////////////////////////////////////////////////////////

void mc1Callback(const std_msgs::Int16& msg)
{
  // stop positive values from being written to the motor controller
  if (mc1_fs_p)
  {
    if ((int)msg.data > 0) return;
  }

  // stop negative values from being written to the motor controller
  if (mc1_fs_n)
  {
    if ((int)msg.data < 0) return;
  }
  
  mc1_val = (int)msg.data;
  // convert message value of [-100, 100] to [1000, 2000]
  mc1.writeMicroseconds(((int)msg.data * 5) + 1500);
}

void mc2Callback(const std_msgs::Int16& msg)
{
  mc2_val = msg.data;
  // convert message value of [-100, 100] to [1000, 2000]
  mc2.writeMicroseconds(((int)msg.data * 5) + 1500);
}

void controlStateCallback(const std_msgs::Int8& msg)
{
  control_state = msg.data;
  
  mc1_fs_p = false;
  mc1_fs_n = false;
  mc2_fs_p = false;
  mc2_fs_n = false;
  
  digitalWrite(RELAY_1, RELAY_STATES[msg.data][0]);
  digitalWrite(RELAY_2, RELAY_STATES[msg.data][1]);
  digitalWrite(RELAY_3, RELAY_STATES[msg.data][2]);
}

