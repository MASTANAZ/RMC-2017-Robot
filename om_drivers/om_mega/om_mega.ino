// om_mega
// 
// CREATED BY HARRIS NEWSTEDER AND BLAKE NAZARIO-CASEY
//
// OPERATION CODE FOR THE ON-BOARD ARDUINO MEGA
// NASA RMC 2017

#include "ir_sense.h"

#include <ros.h>
#include <std_msgs/Int16.h>
#include <Servo.h>

IRSense ir;

ros::NodeHandle nh;

Servo lcv;
Servo rcv;

void lcvCallback(const std_msgs::Int16& lcv_msg)
{
  int write_val = ((int)lcv_msg.data * 5) + 1500;
  if (write_val < 1000) write_val = 1000;
  if (write_val > 2000) write_val = 2000;
  lcv.writeMicroseconds(write_val);
}

void rcvCallback(const std_msgs::Int16& rcv_msg)
{
  int write_val = ((int)rcv_msg.data * 5) + 1500;
  if (write_val < 1000) write_val = 1000;
  if (write_val > 2000) write_val = 2000;
  rcv.writeMicroseconds(write_val);
}

ros::Subscriber<std_msgs::Int16> lcv_sub("lcv", &lcvCallback);
ros::Subscriber<std_msgs::Int16> rcv_sub("rcv", &rcvCallback);

void setup()
{
  nh.initNode();
  nh.subscribe(lcv_sub);
  nh.subscribe(rcv_sub);
  lcv.attach(3);
  rcv.attach(2);

  ir.setSensorPin(8);
  ir.setServoPin(11);
}

void loop()
{
  nh.spinOnce();
  ir.tick();
  delay(5);
}
