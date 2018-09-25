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
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/FluidPressure.h>
#include "comm_stereo_hlc/FeaturePoints.h"

#include "UdpLink.h"

static unsigned char imgBuf[307200] = { 0 };
static unsigned char imgBuf_r[307200] = { 0 };

UdpLink::UdpLink() : fd_(-1), feature_num_(0), seq_num_(0)
{
  ros::NodeHandle private_nh("~");
  private_nh.getParam("udp_port", param_udp_port_);
  private_nh.getParam("upd_ip", param_udp_ipaddr_);
  private_nh.getParam("stereo_publish_freq", param_stereo_publish_freq_);
  mat_left_img_ = cv::Mat(480, 640, CV_8UC1);
  mat_right_img_ = cv::Mat(480, 640, CV_8UC1);

  image_transport::ImageTransport it(private_nh);
  image_transport::ImageTransport it2(private_nh);
  left_img_pub_ = it.advertise("left/image_raw", 10);
  right_img_pub_ = it2.advertise("right/image_raw", 10);
  left_img_out_pub_ = it.advertise("left_out/image_raw", 10);
  right_img_out_pub_ = it.advertise("right_out/image_raw", 10);

  feature_pub_1_ = private_nh.advertise<comm_stereo_hlc::FeaturePoints>("feature_1", 10);
  feature_pub_2_ = private_nh.advertise<comm_stereo_hlc::FeaturePoints>("feature_2", 10);
  feature_pub_3_ = private_nh.advertise<comm_stereo_hlc::FeaturePoints>("feature_3", 10);
  feature_pub_4_ = private_nh.advertise<comm_stereo_hlc::FeaturePoints>("feature_4", 10);

  imu_pub_ = private_nh.advertise<sensor_msgs::Imu>("adis16448/Imu", 10);
  mag_pub_ = private_nh.advertise<sensor_msgs::MagneticField>("adis16448/Mag", 10);
  pressure_pub_ = private_nh.advertise<sensor_msgs::FluidPressure>("adis16448/Pressure", 10);
  temp_pub_ = private_nh.advertise<sensor_msgs::Temperature>("adis16448/Temperature", 10);
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
//  fcntl(fd_, F_SETFL, O_NONBLOCK);

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
    int nrcv = recvfrom(fd_, buffer, SYNC_DATA_LEN, 0, reinterpret_cast<sockaddr*>(&from), &len_from);
    if (nrcv == -1)
    {
      //    ROS_INFO("[UdpLink] recvfrom failed: %s\n", strerror(errno));
      return;
    }
    else
    {
      //      ROS_INFO("[UdpLink] recvfrom %d bytes\n", nrcv);

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

  comm_stereo_hlc::FeaturePoints feature_msg;
  sensor_msgs::Imu adis16448_imu_msg;
  sensor_msgs::FluidPressure adis16448_pressure_msg;
  sensor_msgs::MagneticField adis16448_mag_msg;
  sensor_msgs::Temperature adis16448_temp_msg;

  bool sync_flag = false;
  if (/*len == STEREO_IMAGE_LEN ||*/ len == SYNC_DATA_LEN)
  {
    // Decode left image first (buffer[1281] = 1)
    if ((unsigned char)buffer[1281] == 1)
    {
      if ((unsigned char)buffer[1280] == 0xAA)
      {
        flag_left = true;
        memcpy(imgBuf + index * 1280, buffer, 1280);
        index++;

        seq_num_ = (uint16_t &)buffer[1283];
        ROS_INFO("Start of left stereo image data, seq %d\n", seq_num_);
        time_stamp_left_ = (long &)buffer[1286];
      }
      else if ((unsigned char)buffer[1280] == 0x00 /*&& flag_left*/)
      {
        unsigned int seq_num = (uint16_t &)buffer[1283];
        if (seq_num != seq_num_)
        {
//          ROS_INFO("Seq num mismatch in left image 0x00.\n");
          return;
        }

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
        static unsigned int left_img_cnt = 0;

        unsigned int seq_num = (uint16_t &)buffer[1283];
        if (seq_num != seq_num_)
        {
          ROS_INFO("Seq num mismatch in left image 0xBB.\n");
          return;
        }

//        long time_stamp_left = (long &)buffer[1286];
        ROS_INFO("End of this left frame image data, index %d, time %lu\n", index, time_stamp_left_);

        memcpy(imgBuf + index * 1280, buffer, 1280);
        mat_left_img_.data = imgBuf;

        flag_left = false;
        index = 0;

        for (std::vector<IMU_TIMESTAMP>::iterator it = imu_time_stamp_vec_.begin(); it != imu_time_stamp_vec_.end(); it++)
        {
          if (it->fpga_timestamp == time_stamp_left_)
          {
            // Timestamp found, publish the left image
            ROS_INFO("Publish left image at %lu", time_stamp_left_);
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", mat_left_img_).toImageMsg();
            msg->header.stamp = it->ros_timestamp;
            left_img_pub_.publish(msg);
            break;
          }
        }
        // Publish image topic
//        if ()

      }
    }
    else if ((unsigned char)buffer[1281] == 2)  // Decode right image
    {
      //      index_r = (uint8_t &)buffer[1282];
      if ((unsigned char)buffer[1280] == 0xAA)
      {
        unsigned int seq_num = (uint16_t &)buffer[1283];
        if (seq_num != seq_num_)
        {
          ROS_INFO("Seq num mismatch in right image 0xAA.\n");
          return;
        }

        flag_right = true;
        ROS_INFO("Start of right stereo image data.\n");
        time_stamp_right_ = (long &)buffer[1286];
        memcpy(imgBuf_r + index_r * 1280, buffer, 1280);
        index_r++;
      }
      else if ((unsigned char)buffer[1280] == 0x00 /*&& flag_right*/)
      {
        unsigned int seq_num = (uint16_t &)buffer[1283];
        if (seq_num != seq_num_)
        {
//          ROS_INFO("Seq num mismatch in right image 0x00.\n");
          return;
        }

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
        unsigned int seq_num = (uint16_t &)buffer[1283];
        if (seq_num != seq_num_)
        {
          ROS_INFO("Seq num mismatch in right image 0xBB.\n");
          return;
        }

        ROS_INFO("End of this right frame image data, index %d\n", index_r);
        memcpy(imgBuf_r + index_r * 1280, buffer, 1280);
        mat_right_img_.data = imgBuf_r;

        flag_right = false;
        index_r = 0;

        for (std::vector<IMU_TIMESTAMP>::iterator it = imu_time_stamp_vec_.begin(); it != imu_time_stamp_vec_.end(); it++)
        {
          if (it->fpga_timestamp == time_stamp_right_)
          {
            // Publish image topic
            ROS_INFO("Publish right image at %lu", time_stamp_right_);
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", mat_right_img_).toImageMsg();
            msg->header.stamp = it->ros_timestamp;
            right_img_pub_.publish(msg);
            break;
          }
        }
      }
    }
    else if ((unsigned char)buffer[1281] == 3 && (unsigned char)buffer[1280] == 1)
    {
      unsigned int seq_num = (uint16_t &)buffer[1284];
      if (seq_num != seq_num_)
      {
        ROS_INFO("Seq num mismatch in feature1, %d.\n", seq_num);
        return;
      }

      // Decode feature points for previous left image - feature_1
      feature_num_ = (uint8_t&)buffer[1282];
      ROS_INFO("Feature points %d\n", feature_num_);
      if (feature_num_ <= 255)
      {
        memcpy(feature_pnts_arry_, (unsigned char*)buffer, sizeof(FEATURE_POINT) * feature_num_);
        feature_msg.num = feature_num_;
      }
      else
      {
        ROS_INFO("Incorrect number of feature points.\n");
        return;
      }

      int16_t* p_feature = feature_pnts_arry_;
      for (unsigned int i = 0; i < feature_num_; i++)
      {
        feature_pnts_vec_.push_back(FEATURE_POINT(*p_feature, *(p_feature + 1)));
        feature_msg.datax.push_back(*p_feature);
        feature_msg.datay.push_back(*(p_feature + 1));
        p_feature += 2;
      }

//                  displayFeaturePoints();

      // Publish feature_1 topic
      feature_pub_1_.publish(feature_msg);

      // Display feature points on stereo images
      // Draw on left previous
      for (std::vector<FEATURE_POINT>::iterator it = feature_pnts_vec_.begin(); it != feature_pnts_vec_.end(); it++)
      {
        cv::drawMarker(mat_left_img_, cv::Point(it->x1, it->y1), cv::Scalar(255, 0, 255), cv::MARKER_CROSS);
      }

      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", mat_left_img_).toImageMsg();
      left_img_out_pub_.publish(msg);

      // Clear the feature points
      feature_pnts_vec_.clear();
    }

    else if ((unsigned char)buffer[1281] == 3 && (unsigned char)buffer[1280] == 2)
    {
      unsigned int seq_num = (uint16_t &)buffer[1284];
      if (seq_num != seq_num_)
      {
        ROS_INFO("Seq num mismatch in feature2, %d.\n", seq_num);
        return;
      }

      // Decode feature points for previous right image
      feature_num_ = (uint8_t&)buffer[1282];
      ROS_INFO("Feature points %d\n", feature_num_);
      if (feature_num_ <= 255)
      {
        memcpy(feature_pnts_arry_, (unsigned char*)buffer, sizeof(FEATURE_POINT) * feature_num_);
        feature_msg.num = feature_num_;
      }
      else
      {
        ROS_INFO("Incorrect number of feature points.\n");
        return;
      }

      int16_t* p_feature = feature_pnts_arry_;
      for (unsigned int i = 0; i < feature_num_; i++)
      {
        feature_pnts_vec_.push_back(FEATURE_POINT(*p_feature, *(p_feature + 1)));
        feature_msg.datax.push_back(*p_feature);
        feature_msg.datay.push_back(*(p_feature + 1));
        p_feature += 2;
      }

      //            displayFeaturePoints();

      // Publish feature_2 topic
      feature_pub_2_.publish(feature_msg);

      // Display feature points on stereo images
      // Draw on left previous
      for (std::vector<FEATURE_POINT>::iterator it = feature_pnts_vec_.begin(); it != feature_pnts_vec_.end(); it++)
      {
        cv::drawMarker(mat_right_img_, cv::Point(it->x1, it->y1), cv::Scalar(255, 0, 255), cv::MARKER_CROSS);
      }

      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", mat_right_img_).toImageMsg();
      right_img_out_pub_.publish(msg);

      // Clear the feature points
      feature_pnts_vec_.clear();
    }
    else if ((unsigned char)buffer[1281] == 3 && (unsigned char)buffer[1280] == 3)
    {
      unsigned int seq_num = (uint16_t &)buffer[1284];
      if (seq_num != seq_num_)
      {
        ROS_INFO("Seq num mismatch in feature3, %d.\n", seq_num);
        return;
      }

      // Decode feature points for current right image
      feature_num_ = (uint8_t&)buffer[1282];
      ROS_INFO("Feature points %d\n", feature_num_);
      if (feature_num_ <= 255)
      {
        memcpy(feature_pnts_arry_, (unsigned char*)buffer, sizeof(FEATURE_POINT) * feature_num_);
        feature_msg.num = feature_num_;
      }
      else
      {
        ROS_INFO("Incorrect number of feature points.\n");
        return;
      }

      int16_t* p_feature = feature_pnts_arry_;
      for (unsigned int i = 0; i < feature_num_; i++)
      {
        feature_pnts_vec_.push_back(FEATURE_POINT(*p_feature, *(p_feature + 1)));
        feature_msg.datax.push_back(*p_feature);
        feature_msg.datay.push_back(*(p_feature + 1));
        p_feature += 2;
      }

      //            displayFeaturePoints();

      // Publish feature_3 topic
      feature_pub_3_.publish(feature_msg);

      // Clear the feature points
      feature_pnts_vec_.clear();
    }
    else if ((unsigned char)buffer[1281] == 3 && (unsigned char)buffer[1280] == 4)
    {
      unsigned int seq_num = (uint16_t &)buffer[1284];
      if (seq_num != seq_num_)
      {
        ROS_INFO("Seq num mismatch in feature4, %d.\n", seq_num);
        return;
      }

      // Decode feature points for current left image
      feature_num_ = (uint8_t&)buffer[1282];
      ROS_INFO("Feature points %d\n", feature_num_);
      if (feature_num_ <= 255)
      {
        memcpy(feature_pnts_arry_, (unsigned char*)buffer, sizeof(FEATURE_POINT) * feature_num_);
        feature_msg.num = feature_num_;
      }
      else
      {
        ROS_INFO("Incorrect number of feature points.\n");
        return;
      }

      int16_t* p_feature = feature_pnts_arry_;
      for (unsigned int i = 0; i < feature_num_; i++)
      {
        feature_pnts_vec_.push_back(FEATURE_POINT(*p_feature, *(p_feature + 1)));
        feature_msg.datax.push_back(*p_feature);
        feature_msg.datay.push_back(*(p_feature + 1));
        p_feature += 2;
      }

      //            displayFeaturePoints();

      // Publish feature_4 topic
      feature_pub_4_.publish(feature_msg);

      // Clear the feature points
      feature_pnts_vec_.clear();
    }
    else if ((unsigned char)buffer[1281] == 4 /* IMU data */)
    {
      static unsigned int imu_cnt = 0;
      // Decode sync data
      long time_stamp_imu = (long &)buffer[1286];

      ROS_INFO("IMU data, %lu\n", time_stamp_imu);

      IMU_TIMESTAMP timestamp = {time_stamp_imu, ros::Time::now()};
      imu_time_stamp_vec_.push_back(timestamp);

      unsigned char imu_buf[22] = {0};
      memcpy(imu_buf, (unsigned char*)buffer, sizeof(imu_buf));

      float xgyro = *(short *)(imu_buf + 0) * IMU_GYRO_FAC;
      float ygyro = *(short *)(imu_buf + 2) * IMU_GYRO_FAC;
      float zgyro = *(short *)(imu_buf + 4) * IMU_GYRO_FAC;

      float xacc = *(short *)(imu_buf + 6) * IMU_ACCL_MS2_FAC;
      float yacc = *(short *)(imu_buf + 8) * IMU_ACCL_MS2_FAC;
      float zacc = *(short *)(imu_buf + 10) * IMU_ACCL_MS2_FAC;

      float xmag = *(short *)(imu_buf + 12) * IMU_MAGN_FAC;
      float ymag = *(short *)(imu_buf + 14) * IMU_MAGN_FAC;
      float zmag = *(short *)(imu_buf + 16) * IMU_MAGN_FAC;

      float baro = *(unsigned short *)(imu_buf + 18) * IMU_BARO_KPA_FAC;

      unsigned short temp_buf;
      memcpy(&temp_buf, imu_buf + 20, sizeof(temp_buf));
      float temp = 0.0f;
      if (temp_buf & (1 << 11))
      {
        // MSB11 is 1
        temp = ((short)temp_buf - 4096) * IMU_TEMP_FAC;
      }
      else
      {
        // MSB11 is 0
        temp = (short)(temp_buf) * IMU_TEMP_FAC;
      }
      temp += 31.0f;

      ros::Time imu_ros_timestamp = timestamp.ros_timestamp;
      adis16448_imu_msg.header.stamp = imu_ros_timestamp;
      adis16448_imu_msg.linear_acceleration.x = xacc;
      adis16448_imu_msg.linear_acceleration.y = yacc;
      adis16448_imu_msg.linear_acceleration.z = zacc;
      adis16448_imu_msg.angular_velocity.x = xgyro;
      adis16448_imu_msg.angular_velocity.y = ygyro;
      adis16448_imu_msg.angular_velocity.z = zgyro;

      adis16448_mag_msg.header.stamp = imu_ros_timestamp;
      adis16448_mag_msg.magnetic_field.x = xmag / 1e5;
      adis16448_mag_msg.magnetic_field.y = ymag / 1e5;
      adis16448_mag_msg.magnetic_field.z = zmag / 1e5;

      adis16448_pressure_msg.header.stamp = imu_ros_timestamp;
      adis16448_pressure_msg.fluid_pressure = baro;

      adis16448_temp_msg.header.stamp = imu_ros_timestamp;
      adis16448_temp_msg.temperature = temp;

      imu_pub_.publish(adis16448_imu_msg);
      mag_pub_.publish(adis16448_mag_msg);
      pressure_pub_.publish(adis16448_pressure_msg);
      temp_pub_.publish(adis16448_temp_msg);

//      ROS_INFO("Publish IMU data");
      printf("IMU time stamp window size %d\n", imu_time_stamp_vec_.size());
      if (imu_time_stamp_vec_.size() == IMU_TIMESTAMP_WINDOW_SIZE)
      {
        imu_time_stamp_vec_.clear();
      }

      if (imu_cnt++ % 20 == 0) {
        printf("Gyro: x %.3f, y %.3f, z %.3f\n", xgyro, ygyro, zgyro);
        printf("Acc: x %.3f, y %.3f, z %.3f\n", xacc, yacc, zacc);
        printf("Mag: x %.3f, y %.3f, z %.3f\n", xmag, ymag, zmag);
        printf("baro: %.3f, temp: %.3f \n", baro, temp);
      }
    }

  }
}

void UdpLink::displayFeaturePoints()
{
  for (std::vector<FEATURE_POINT>::iterator it = feature_pnts_vec_.begin(); it != feature_pnts_vec_.end(); it++)
  {
    printf("%d %d \n", it->x1, it->y1);
  }
}
