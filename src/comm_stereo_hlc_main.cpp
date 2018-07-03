/*
 * comm_stereo_hlc_main.cpp
 *
 *  Created on: 31 May 2017
 *      Author: nusuav
 */

#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <vector>
#include "UdpLink.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "comm_stereo_hlc");
  ros::NodeHandle nh("~");
  ros::Rate loop_rate(100);

  ROS_INFO("comm_stereo_hlc start\n");

  UdpLink udplink;
  if (!udplink.init())
    return false;

  while (ros::ok())
  {
    udplink.mainLoop();

    ros::spinOnce();
    //            loop_rate.sleep();
    //    ros::spinOnce();
    //    usleep(2);
  }
}
