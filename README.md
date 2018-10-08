# Introduction

This package is developed to realize the communication between zynq-based stereo board and high level computer such as UP board, NUC, etc.. The current communication interface is via Ethernet port with customized protocol sent from FPGA.  
The data will contain stereo images, feature points and ADIS16448 IMU output. 

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites

Ubuntu 16.04 with ROS Kinetic.

### Building

```
catkin_make
```
## ROS API


#### Published topics

`/comm_stereo_hlc/left/image_raw` ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))  
`/comm_stereo_hlc/right/image_raw` ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))  
`/comm_stereo_hlc/adis16448/Imu` ([sensor_msgs/Imu](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html))  
`/comm_stereo_hlc/adis16448/Mag` ([sensor_msgs/MagneticField](http://docs.ros.org/api/sensor_msgs/html/msg/MagneticField.html))  
`/comm_stereo_hlc/adis16448/Pressure` ([sensor_msgs/FluidPressure](http://docs.ros.org/api/sensor_msgs/html/msg/FluidPressure.html))  
`/comm_stereo_hlc/adis16448/Temperature` ([sensor_msgs/Temperature](http://docs.ros.org/api/sensor_msgs/html/msg/Temperature.html))  
## Authors

* **Xiangxu Dong** - tsldngx@nus.edu.sg
