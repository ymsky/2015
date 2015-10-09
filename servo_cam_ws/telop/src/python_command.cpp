/*
 * python_command.cpp
 *
 *  Created on: Sep 26, 2015
 *      Author: s
 */


#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <iostream>
#include <sstream>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/UInt16.h"


int m_index=1;

void callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
	if(msg->data.size())
	{

		std::vector<float>::const_iterator it=msg->data.begin();


		float objectID;
		float dxToCenter;
		float dyToCenter;
		objectID=*it;
		it=it+9;
		dxToCenter=*it;
		it++;
		dyToCenter=*it;

		float gap=50;

		float dx=dxToCenter-320;
		float dy=dyToCenter-240;
		float slope = dy/dx;
		if((slope<=-2.414)||(slope>=2.414))
		{
			if(dy>0)	{m_index=2;ROS_INFO("2,3");}
			else		{m_index=0;ROS_INFO("0,1");}
		}
		else if ((slope>0.414)&&(slope<2.414))
		{
			if(dx>0)	{m_index=93;ROS_INFO("93,1");}
			else		{m_index=90;ROS_INFO("90,1");}
		}
		else if ((slope>=-0.414)&&(slope<=0.414))
		{
			if(dx>0)	{m_index=6;ROS_INFO("6,7");}
			else		{m_index=4;ROS_INFO("4,5");}
		}
		else
		{
			if(dx>0)	{m_index=91;ROS_INFO("91,1");}
			else		{m_index=92;ROS_INFO("92,1");}
		}

		if	((dx<gap)&&(dx>-gap)&&(dy<gap)&&(dy>-gap)) {m_index=1;ROS_INFO("CENTER");}


		ROS_INFO("%f %f %f", objectID, dxToCenter, dyToCenter);
	}
	else
	{
		ROS_INFO("no object");
	}


}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "python_command");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<std_msgs::UInt16>("command_request", 2);
	ros::Subscriber sub = nh.subscribe("/objects", 1, callback);

	ros::Rate loop_rate(2);

	int counter;

	counter =1;
	while (ros::ok())
	{
		std_msgs::UInt16 msg_command_index;
		counter=-counter;
		if (counter==-1) m_index=1;

		msg_command_index.data=m_index;
		pub.publish(msg_command_index);
		ros::spinOnce();
		loop_rate.sleep();

	}

}
