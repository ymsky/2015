/*
 * hvd_control.cpp
 *
 *  Created on: Nov 15, 2015
 *      Author: s
 */

#include "ros/ros.h"
#include <sensor_msgs/Joy.h>

#include <std_msgs/UInt16.h>


#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
using namespace std;


class HDVControl
{
public:
  HDVControl();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int command_index_;

  ros::Publisher command_pub_;
  ros::Subscriber joy_sub_;

};

HDVControl::HDVControl():
  command_index_(0)
{

  command_pub_ = nh_.advertise<std_msgs::UInt16>("command_pub", 1);


  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &HDVControl::joyCallback, this);

}


void HDVControl::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	std_msgs::UInt16 command_msg;

	if (joy->buttons[0]==1)
	{
		//stop
		cout<<"stop"<<endl;
		command_index_=0;
	}
	if (joy->buttons[1]==1)
	{
		cout<<"straight"<<endl;
		command_index_=1;
	}
	if (joy->buttons[2]==1)
	{
		cout<<"rotate"<<endl;
		command_index_=2;
	}
	command_msg.data=command_index_;
	command_pub_.publish(command_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hdv_control");
  HDVControl hc;

  ros::spin();
}
