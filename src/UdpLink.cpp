/*
 * UdpLink.cpp
 *
 *  Created on: 31 May 2018
 *      Author: nusuav
 */

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include "UdpLink.h"

static unsigned char imgBuf[307200] = { 0 };
static unsigned char imgBuf_r[307200] = { 0 };

UdpLink::UdpLink() : fd_(-1)
{
  ros::NodeHandle private_nh("~");
  private_nh.getParam("udp_port", param_udp_port_);
  private_nh.getParam("upd_ip", param_udp_ipaddr_);

  mat_left_img_ = cv::Mat(480, 640, CV_8UC1);
  mat_right_img_ = cv::Mat(480, 640, CV_8UC1);

  image_transport::ImageTransport it(private_nh);
  image_transport::ImageTransport it2(private_nh);
  left_img_pub_ = it.advertise("left_image", 1);
  right_img_pub_ = it2.advertise("right_image", 1);
}

UdpLink::~UdpLink()
{
  close(fd_);
}

bool UdpLink::init()
{
  fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd_ == -1)
  {
    ROS_INFO("[UdpLink] create socket failed: %s\n", strerror(errno));
    return false;
  }

  // Set the socket file descriptor to non-blocking
  fcntl(fd_, F_SETFL, O_NONBLOCK);

  // Bind to the socket with IP and port
  sockaddr_in addr;
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = inet_addr(param_udp_ipaddr_.c_str());
  addr.sin_port = htons(param_udp_port_);
  memset(addr.sin_zero, 0, sizeof(addr.sin_zero));

  if (bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == -1)
  {
    ROS_INFO("[UdpLink] bind failed: %s\n", strerror(errno));
    return false;
  }

  return true;
}

void UdpLink::mainLoop()
{
  // Receive udp data packets and parse
  struct sockaddr_in from;
  unsigned int len_from = sizeof(from);

  char buffer[2048];

  FD_ZERO(&readfds_);
  FD_SET(fd_, &readfds_);  // Turn on the bit corresponding to the socket to check readability

  struct timeval tv = { 0, 0 };
  int ret = select(fd_ + 1, &readfds_, NULL, NULL, &tv);

  if (ret == -1)
  {
    printf("select %s\n", strerror(errno));
    return;
  }
  else if (ret == 1)
  {
    int nrcv = recvfrom(fd_, buffer, 1288, 0, reinterpret_cast<sockaddr*>(&from), &len_from);
    if (nrcv == -1)
    {
      //    ROS_INFO("[UdpLink] recvfrom failed: %s\n", strerror(errno));
      return;
    }
    else
    {
      ROS_INFO("[UdpLink] recvfrom %d bytes\n", nrcv);

      // Process data
      processData(nrcv, buffer);
    }
  }
  else
  {
    //  printf("Nothing to do.\n");
  }
}

void UdpLink::processData(unsigned int len, char* buffer)
{
  static unsigned int index = 0, index_r = 0;
  static bool flag_left = false, flag_right = false;
  if (len == STEREO_IMAGE_LEN)
  {
    if ((unsigned char)buffer[1281] == 1)
    {
      if ((unsigned char)buffer[1280] == 0xAA)
      {
        flag_left = true;
        ROS_INFO("Start of left stereo image data.\n");
        memcpy(imgBuf + index * 1280, buffer, 1280);
      }
      else if ((unsigned char)buffer[1280] == 0x00 && flag_left)
      {
        if (index <= 238)
        {
          memcpy(imgBuf + index * 1280, buffer, 1280);
          index++;
        }
        else
        {
          // Corrupt packet
          flag_left = false;
          index = 0;
        }
      }
      else if ((unsigned char)buffer[1280] == 0xBB && flag_left)
      {
        ROS_INFO("End of this frame image data, index %d\n", index);
        memcpy(imgBuf + index * 1280, buffer, 1280);
        mat_left_img_.data = imgBuf;

        flag_left = false;
        index = 0;

        // Publish image topic
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", mat_left_img_).toImageMsg();
        left_img_pub_.publish(msg);
      }
    }
    else if ((unsigned char)buffer[1281] == 2)
    {
      if ((unsigned char)buffer[1280] == 0xAA)
      {
        flag_right = true;
        ROS_INFO("Start of right stereo image data.\n");
        memcpy(imgBuf_r + index_r * 1280, buffer, 1280);
      }
      else if ((unsigned char)buffer[1280] == 0x00 && flag_right)
      {
        if (index <= 238)
        {
          memcpy(imgBuf_r + index_r * 1280, buffer, 1280);
          index_r++;
        }
        else
        {
          // Corrupt packet
          flag_right = false;
          index_r = 0;
        }
      }
      else if ((unsigned char)buffer[1280] == 0xBB && flag_right)
      {
        ROS_INFO("End of this frame image data, index %d\n", index_r);
        memcpy(imgBuf_r + index_r * 1280, buffer, 1280);
        mat_right_img_.data = imgBuf_r;

        flag_right = false;
        index_r = 0;

        // Publish image topic
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", mat_right_img_).toImageMsg();
        right_img_pub_.publish(msg);
      }
    }

    // Next stereo right image
  }

  // Feature points

  // Estimated pose
}
