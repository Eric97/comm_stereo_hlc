/*
 * UdpLink.h
 *
 *  Created on: 31 May 2018
 *      Author: nusuav
 */

#ifndef COMM_STEREO_HLC_SRC_UDPLINK_H_
#define COMM_STEREO_HLC_SRC_UDPLINK_H_

#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <string>

class UdpLink
{
public:
  UdpLink();
  ~UdpLink();

  static const unsigned int STEREO_IMAGE_LEN = 1288;

private:
  int fd_;          ///< Socket descriptor
  fd_set readfds_;  ///< File descriptor

  // Parameters
  int param_udp_port_;            //!< UDP port address to receive data from stereo board
  std::string param_udp_ipaddr_;  //!< IP address to receive data from stereo board

  cv::Mat mat_left_img_, mat_right_img_;  //!< Left and right images in OpenCV format

  ros::NodeHandle node_;
  // ros::Publisher left_img_pub_;	//!< Left image topic publisher
  image_transport::Publisher left_img_pub_, right_img_pub_;  //!< Left and right image topic publisher

public:
  bool init();
  void mainLoop();
  void processData(unsigned int nrcv, char* buffer);
};

#endif /* COMM_STEREO_HLC_SRC_UDPLINK_H_ */
