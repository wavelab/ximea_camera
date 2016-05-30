/******************************************************************************

Copyright 2015  Abdelhamid El-Bably (University of Waterloo)
                      [ahelbably@uwaterloo.ca]
                Arun Das (University of Waterloo)
                      [adas@uwaterloo.ca]
                Michael Tribou (University of Waterloo)
                      [mjtribou@uwaterloo.ca]

All rights reserved.

********************************************************************************/

#ifndef XIMEA_CAMERA_XIMEA_ROS_DRIVER_H
#define XIMEA_CAMERA_XIMEA_ROS_DRIVER_H

#include <ximea_camera/ximea_driver.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <string>
#include <boost/shared_ptr.hpp>

class ximea_ros_driver : public ximea_driver
{
public:
  ximea_ros_driver(const ros::NodeHandle &nh, std::string cam_name, int serial_no , std::string yaml_url);
  ximea_ros_driver(const ros::NodeHandle &nh, std::string file_name);
  virtual void setImageDataFormat(std::string s);
  void publishImage(const ros::Time & now);  // since these 2 functions should have the same time stamp we leave it up to the user to specify the timeif it is needed to do one or the other
  void publishCamInfo(const ros::Time &now);
  void publishImageAndCamInfo();

protected:
  ros::NodeHandle pnh_;
  boost::shared_ptr<camera_info_manager::CameraInfoManager> cam_info_manager_;
  boost::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::Publisher ros_cam_pub_;
  ros::Publisher cam_info_pub_;

  sensor_msgs::Image ros_image_;
  sensor_msgs::CameraInfo cam_info_;
  char * cam_buffer_;
  int cam_buffer_size_;
  int bpp_;  // the next 2 paramaeters are used by the ros_image_transport publisher
  std::string encoding_;

private:
  void common_initialize(const ros::NodeHandle &nh);
};

#endif  // XIMEA_CAMERA_XIMEA_ROS_DRIVER_H
