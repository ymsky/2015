/*
 * opencv.cpp
 *
 *  Created on: Sep 26, 2015
 *      Author: s
 */

#include <ros/ros.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include <stdio.h>
#include <iostream>
//#include <std::string.h>

int main(int argc, char **argv) {

	ros::init(argc, argv, "opencv_pub_ros_image");
    cv::Mat image;
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::Image>("image_out", 2);

    cv_bridge::CvImage img;
    img.encoding = "bgr8";
    img.header.stamp = ros::Time::now();
    img.header.frame_id = "whatever_frame";

    const std::string camera_address="http://192.168.1.37:80/videostream.cgi?loginuse=admin&loginpas=12345&.mjpg";
   cv::VideoCapture vcap(camera_address);

//    cv::VideoCapture vcap;
 //   vcap.open(1);
    if (!vcap.isOpened())
    {
    	ROS_INFO("video capture failed");
    }
    else
    {
    	ROS_INFO("video capture success");
    	vcap >> image;
    	if(image.empty()) ROS_INFO("capture empty video");
    	else
    	{
    		ROS_INFO("video capture is working");
    		while(ros::ok())
    		{

    		    vcap >> image;
    		    img.image = image;
    		    pub.publish(img.toImageMsg());
    		}
    	}


    }


}
