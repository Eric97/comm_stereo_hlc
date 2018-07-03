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

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "UdpLink.h"

static unsigned char imgBuf[307200] = { 0 };
static unsigned char imgBuf_r[307200] = { 0 };

UdpLink::UdpLink() : fd_(-1), feature_num_(0)
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
  left_img_out_pub_ = it.advertise("left_out_image", 1);
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

  int nrcv = recvfrom(fd_, buffer, 1288, 0, reinterpret_cast<sockaddr*>(&from), &len_from);
  //  ROS_INFO("[UdpLink] recvfrom %d bytes\n", nrcv);
  processData(nrcv, buffer);

  //  FD_ZERO(&readfds_);
  //  FD_SET(fd_, &readfds_);  // Turn on the bit corresponding to the socket to check readability
  //
  //  struct timeval tv = { 0, 10 };
  //  int ret = select(fd_ + 1, &readfds_, NULL, NULL, &tv);
  //
  //  if (ret == -1)
  //  {
  //    printf("select %s\n", strerror(errno));
  //    return;
  //  }
  //  else if (ret == 1)
  //  {
  //    int nrcv = recvfrom(fd_, buffer, 1288, 0, reinterpret_cast<sockaddr*>(&from), &len_from);
  //    if (nrcv == -1)
  //    {
  //      //    ROS_INFO("[UdpLink] recvfrom failed: %s\n", strerror(errno));
  //      return;
  //    }
  //    else
  //    {
  ////      ROS_INFO("[UdpLink] recvfrom %d bytes\n", nrcv);
  //
  //      // Process data
  //      processData(nrcv, buffer);
  //    }
  //  }
  //  else
  //  {
  //    //  printf("Nothing to do.\n");
  //  }
}

void UdpLink::processData(unsigned int len, char* buffer)
{
  static unsigned int index = 0, index_r = 0;
  static bool flag_left = false, flag_right = false;
  if (len == STEREO_IMAGE_LEN)
  {
    // Decode left image first (buffer[1281] = 1)
    if ((unsigned char)buffer[1281] == 1)
    {
      if ((unsigned char)buffer[1280] == 0xAA)
      {
        flag_left = true;
        ROS_INFO("Start of left stereo image data.\n");
        memcpy(imgBuf + index * 1280, buffer, 1280);
        index++;
      }
      else if ((unsigned char)buffer[1280] == 0x00 /*&& flag_left*/)
      {
        if (index <= 239)
        {
          memcpy(imgBuf + index * 1280, buffer, 1280);
          index++;
        }
        else
        {
          // Corrupt packet
          //          flag_left = false;
          index = 0;
        }
      }
      else if ((unsigned char)buffer[1280] == 0xBB /*&& flag_left /*&& ((uint8_t &)buffer[1282] == 239)*/)
      {
        ROS_INFO("End of this left frame image data, index %d\n", index);
        memcpy(imgBuf + index * 1280, buffer, 1280);
        mat_left_img_.data = imgBuf;

        flag_left = false;
        index = 0;

        // Publish image topic
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", mat_left_img_).toImageMsg();
        left_img_pub_.publish(msg);
      }
    }
    else if ((unsigned char)buffer[1281] == 2)  // Decode right image
    {
      //      index_r = (uint8_t &)buffer[1282];
      if ((unsigned char)buffer[1280] == 0xAA)
      {
        flag_right = true;
        ROS_INFO("Start of right stereo image data.\n");
        memcpy(imgBuf_r + index_r * 1280, buffer, 1280);
        index_r++;
      }
      else if ((unsigned char)buffer[1280] == 0x00 /*&& flag_right*/)
      {
        if (index_r <= 239)
        {
          memcpy(imgBuf_r + index_r * 1280, buffer, 1280);
          index_r++;
        }
        else
        {
          // Corrupt packet
          //          flag_right = false;
          index_r = 0;
        }
      }
      else if ((unsigned char)buffer[1280] == 0xBB /*&& flag_right /*&& ((unsigned char)buffer[1282] == 239)*/)
      {
        ROS_INFO("End of this right frame image data, index %d\n", index_r);
        memcpy(imgBuf_r + index_r * 1280, buffer, 1280);
        mat_right_img_.data = imgBuf_r;

        flag_right = false;
        index_r = 0;

        // Publish image topic
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", mat_right_img_).toImageMsg();
        right_img_pub_.publish(msg);
      }
    }
    else if ((unsigned char)buffer[1281] == 3 && (unsigned char)buffer[1280] == 1)
    {
      static unsigned int loop = 0;
      // Decode feature points for previous left image
      feature_num_ = (uint8_t&)buffer[1282];
      ROS_INFO("Feature points %d\n", feature_num_);
      if (feature_num_ <= 255)
      {
        memcpy(feature_pnts_arry_, (unsigned char*)buffer, sizeof(FEATURE_POINT) * feature_num_);
      }
      else
      {
        ROS_INFO("Incorrect number of feature points.\n");
        return;
      }

      int16_t* p_feature = feature_pnts_arry_;
      for (unsigned int i = 0; i < feature_num_; i++)
      {
        feature_pnts_vec_.push_back(FEATURE_POINT(*feature_pnts_arry_, *(feature_pnts_arry_ + 1),
                                                  *(feature_pnts_arry_ + 2), *(feature_pnts_arry_ + 3),
                                                  *(feature_pnts_arry_ + 4), *(feature_pnts_arry_ + 5),
                                                  *(feature_pnts_arry_ + 6), *(feature_pnts_arry_ + 7)));
        p_feature += 8;
      }

      //      displayFeaturePoints();

      // Display feature points on stereo images
      // Draw on left previous
      if (loop % 10 == 0)
      {
        std::vector<cv::KeyPoint> keypoints_1;
        cv::Mat mat_out_img = cv::Mat(480, 640, CV_8UC1);

        for (std::vector<FEATURE_POINT>::iterator it = feature_pnts_vec_.begin(); it != feature_pnts_vec_.end(); it++)
        {
          keypoints_1.push_back(cv::KeyPoint((float)it->x1, (float)it->y1, 2));
          cv::drawKeypoints(mat_left_img_, keypoints_1, mat_out_img, cv::Scalar::all(-1),
                            cv::DrawMatchesFlags::DEFAULT);

          sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", mat_out_img).toImageMsg();
          left_img_out_pub_.publish(msg);
        }
      }
      loop++;
    }
    else if ((unsigned char)buffer[1281] == 3 && (unsigned char)buffer[1280] == 2)
    {
      // Decode feature points for previous right image
    }
  }
}

void UdpLink::displayFeaturePoints()
{
  for (std::vector<FEATURE_POINT>::iterator it = feature_pnts_vec_.begin(); it != feature_pnts_vec_.end(); it++)
  {
    printf("%d %d %d %d %d %d %d %d\n", it->x1, it->y1, it->x2, it->y2, it->x3, it->y3, it->x4, it->y4);
  }
}
