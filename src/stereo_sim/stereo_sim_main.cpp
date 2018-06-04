/*
 * stereo_sim_main.cpp
 *
 *  Created on: 1 Jun 2018
 *      Author: nusuav
 */

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <ros/ros.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

static unsigned char buffer[1280];

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "stereo_sim_node");
  ros::NodeHandle nh("~");
  ros::Rate loop_rate(20);

  /*	UdpLink udp_sender;

    if (!udp_sender.init())
    {
      ROS_INFO("udp_sender init failed.\n");
      return 1;
    }*/

  // Load the image with OpenCV
  cv::Mat left_img = cv::imread("/home/nusuav/Project_Xilinx/regado_catkin/devel/lib/comm_stereo_hlc/left_img.bmp",
                                CV_LOAD_IMAGE_UNCHANGED);
  if (!left_img.data)
  {
    ROS_INFO("Empty image!\n");
    return 1;
  }
  else
  {
    // Copy image data to memory
    ROS_INFO("Size of image: cols %d, rows %d, size %d\n", left_img.cols, left_img.rows,
             left_img.dataend - left_img.datastart);
    memcpy(buffer, left_img.data, 1280);
  }

  int fd_sender = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd_sender == -1)
  {
    ROS_INFO("stereo_sim sender create socket failed: %s\n", strerror(errno));
  }

  fcntl(fd_sender, F_SETFL, O_NONBLOCK);

  sockaddr_in to_addr;
  to_addr.sin_family = AF_INET;
  to_addr.sin_addr.s_addr = inet_addr("192.168.0.107");
  to_addr.sin_port = htons(1455);
  memset(to_addr.sin_zero, 0, sizeof(to_addr.sin_zero));

  while (ros::ok())
  {
    // Send the packets
    int nsent = sendto(fd_sender, buffer, sizeof(buffer), 0, reinterpret_cast<sockaddr*>(&to_addr), sizeof(to_addr));
    if (nsent == -1)
    {
      ROS_INFO("sendto failed: %s\n", strerror(errno));
    }
    else
    {
      ROS_INFO("sendto sent %d bytes\n", nsent);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
