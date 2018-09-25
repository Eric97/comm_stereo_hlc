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

#define IMU_GYRO_FAC  6.981317007977e-4   //unit:rad/sec
#define IMU_GYRO_DEG_FAC  0.04            //unit:deg/sec                      //
#define IMU_ACCL_FAC  8.33e-4             //unit:g                        //
#define IMU_ACCL_MS2_FAC 8.33e-4*9.7814   // unit: m/s^2
#define IMU_MAGN_FAC 0.1429e-3            //unit:gauss                      //
#define IMU_MAGN_MG_FAC  0.1429           //unit:mgauss                      //
#define IMU_TEMP_FAC  0.07386             //unit:centigrade                   //
#define IMU_BARO_FAC  0.02e-3             //unit:bar                        //
#define IMU_BARO_KPA_FAC  0.02e-1         // unit: kpa

class UdpLink
{
public:
  UdpLink();
  ~UdpLink();

  static const unsigned int STEREO_IMAGE_LEN = 1288;
  static const unsigned int SYNC_DATA_LEN = 1294;
  static const unsigned int IMU_TIMESTAMP_WINDOW_SIZE = 200;
  struct FEATURE_POINT
  {
    int16_t x1, y1;

    FEATURE_POINT(int16_t x1, int16_t y1) : x1(x1), y1(y1)
    {
    }
  };

  struct IMU_TIMESTAMP
  {
    long fpga_timestamp;
    ros::Time ros_timestamp;
  };

private:
  int fd_;          ///< Socket descriptor
  fd_set readfds_;  ///< File descriptor

  // Parameters
  int param_udp_port_;            //!< UDP port address to receive data from stereo board
  std::string param_udp_ipaddr_;  //!< IP address to receive data from stereo board
  int param_stereo_publish_freq_; //!< Stereo image publish frequency

  uint16_t seq_num_; //!< Sequence number decoded for current set: left, right, feature1, feature2, feature4, feature3
  cv::Mat mat_left_img_, mat_right_img_;  //!< Left and right images in OpenCV format

  unsigned int feature_num_;         //!< Number of feature points sent from Zynq
  int16_t feature_pnts_arry_[2048];  //!< Allocate maximum of 256 * 8 feature points array

  std::vector<FEATURE_POINT> feature_pnts_vec_;  //!< Feature points vector (x1,y1,x2,y2,x3,y3,x4,y4)

  long time_stamp_left_, time_stamp_right_;  //!< FPGA time stamp of left and right images
  std::vector<IMU_TIMESTAMP> imu_time_stamp_vec_;  //!< IMU time stamp buffer window, 20 @ 20 Hz stereo output, 100 @ 4 Hz stereo output

  ros::NodeHandle node_;
  // ros::Publisher left_img_pub_;	//!< Left image topic publisher
  image_transport::Publisher left_img_pub_, right_img_pub_, left_img_out_pub_,
      right_img_out_pub_;  //!< Left and right image topic publisher

  ros::Publisher feature_pub_1_, feature_pub_2_, feature_pub_3_, feature_pub_4_;
  ros::Publisher imu_pub_;  //!< IMU topic publisher
  ros::Publisher mag_pub_;
  ros::Publisher pressure_pub_;
  ros::Publisher temp_pub_;

public:
  bool init();
  void mainLoop();
  void processData(unsigned int nrcv, char* buffer);
  void displayFeaturePoints();
};

#endif /* COMM_STEREO_HLC_SRC_UDPLINK_H_ */
