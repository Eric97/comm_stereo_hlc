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

  struct FEATURE_POINT
  {
    int16_t x1, y1;

    FEATURE_POINT(int16_t x1, int16_t y1) : x1(x1), y1(y1)
    {
    }
  };

private:
  int fd_;          ///< Socket descriptor
  fd_set readfds_;  ///< File descriptor

  // Parameters
  int param_udp_port_;            //!< UDP port address to receive data from stereo board
  std::string param_udp_ipaddr_;  //!< IP address to receive data from stereo board

  uint16_t seq_num_; //!< Sequence number decoded for current set: left, right, feature1, feature2, feature4, feature3
  cv::Mat mat_left_img_, mat_right_img_;  //!< Left and right images in OpenCV format

  unsigned int feature_num_;         //!< Number of feature points sent from Zynq
  int16_t feature_pnts_arry_[2048];  //!< Allocate maximum of 256 * 8 feature points array

  std::vector<FEATURE_POINT> feature_pnts_vec_;  //!< Feature points vector (x1,y1,x2,y2,x3,y3,x4,y4)

  ros::NodeHandle node_;
  // ros::Publisher left_img_pub_;	//!< Left image topic publisher
  image_transport::Publisher left_img_pub_, right_img_pub_, left_img_out_pub_,
      right_img_out_pub_;  //!< Left and right image topic publisher

  ros::Publisher feature_pub_1_, feature_pub_2_, feature_pub_3_, feature_pub_4_;

public:
  bool init();
  void mainLoop();
  void processData(unsigned int nrcv, char* buffer);
  void displayFeaturePoints();
};

#endif /* COMM_STEREO_HLC_SRC_UDPLINK_H_ */
