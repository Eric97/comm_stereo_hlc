/*
 * UdpLink.h
 *
 *  Created on: 31 May 2018
 *      Author: nusuav
 */

#ifndef COMM_STEREO_HLC_SRC_UDPLINK_H_
#define COMM_STEREO_HLC_SRC_UDPLINK_H_

#include <string>
#include <ros/ros.h>

class UdpLink
{
public:
  UdpLink();
  ~UdpLink();

private:
  int fd_;  ///< Socket descriptor

  // Parameters
  int param_udp_port_;            //!< UDP port address to receive data from stereo board
  std::string param_udp_ipaddr_;  //!< IP address to receive data from stereo board

  ros::NodeHandle node_;

public:
  bool init();
  void mainLoop();
};

#endif /* COMM_STEREO_HLC_SRC_UDPLINK_H_ */
