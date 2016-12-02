/******************************************************************************

Copyright 2015  Abdelhamid El-Bably (University of Waterloo)
                      [ahelbably@uwaterloo.ca]
                Arun Das (University of Waterloo)
                      [adas@uwaterloo.ca]
                Michael Tribou (University of Waterloo)
                      [mjtribou@uwaterloo.ca]

All rights reserved.

********************************************************************************/
#include <ximea_camera/ximea_ros_driver.h>
#include <string>
#include <algorithm>
#include <boost/make_shared.hpp>

ximea_ros_driver::ximea_ros_driver(const ros::NodeHandle &nh, std::string cam_name, int serial_no, std::string yaml_url): ximea_driver(serial_no, cam_name)
{
  pnh_ = nh;
  cam_info_manager_ = boost::make_shared<camera_info_manager::CameraInfoManager>(pnh_, cam_name_);
  cam_info_manager_->loadCameraInfo(yaml_url);
  it_ = boost::make_shared<image_transport::ImageTransport>(nh);
  ros_cam_pub_ = it_->advertise(std::string("image_raw"), 1);
  cam_info_pub_ = pnh_.advertise<sensor_msgs::CameraInfo>(std::string("camera_info"), 1);
}

ximea_ros_driver::ximea_ros_driver(const ros::NodeHandle &nh, std::string file_name) : ximea_driver(file_name)
{
  pnh_ = nh;
  cam_info_manager_ = boost::make_shared<camera_info_manager::CameraInfoManager>(pnh_, cam_name_);
  cam_info_manager_->loadCameraInfo(yaml_url_);
  it_ = boost::make_shared<image_transport::ImageTransport>(nh);
  ros_cam_pub_ = it_->advertise(std::string("image_raw"), 1);
  cam_info_pub_ = pnh_.advertise<sensor_msgs::CameraInfo>(std::string("camera_info"), 1);
}

void ximea_ros_driver::common_initialize(const ros::NodeHandle &nh)
{
  pnh_ = nh;
  cam_info_manager_ = boost::make_shared<camera_info_manager::CameraInfoManager>(pnh_, cam_name_);
  cam_info_manager_->loadCameraInfo("");  // TODO: yaml_url
  it_ = boost::make_shared<image_transport::ImageTransport>(nh);
  ros_cam_pub_ = it_->advertise(cam_name_ + std::string("/image_raw"), 1);
  cam_info_pub_ = pnh_.advertise<sensor_msgs::CameraInfo>(cam_name_ + std::string("/camera_info"), 1);
}

void ximea_ros_driver::publishImage(const ros::Time & now)
{
  cam_buffer_ = reinterpret_cast<char *>(image_.bp);
  cam_buffer_size_ = image_.width * image_.height * bpp_;
  ros_image_.data.resize(cam_buffer_size_);
  ros_image_.encoding = encoding_;
  ros_image_.width = image_.width;
  ros_image_.height = image_.height;
  ros_image_.step = image_.width * bpp_;

  copy(reinterpret_cast<char *>(cam_buffer_),
       (reinterpret_cast<char *>(cam_buffer_)) + cam_buffer_size_,
       ros_image_.data.begin());

  ros_cam_pub_.publish(ros_image_);
}

void ximea_ros_driver::publishCamInfo(const ros::Time &now)
{
  ros_image_.header.stamp = now;
  cam_info_ = cam_info_manager_->getCameraInfo();
  cam_info_.header.frame_id = frame_id_;
  cam_info_pub_.publish(cam_info_);
}

void ximea_ros_driver::publishImageAndCamInfo()
{
  ros::Time now = ros::Time::now();
  publishImage(now);
  publishCamInfo(now);
}

void ximea_ros_driver::setImageDataFormat(std::string image_format)
{
  XI_RETURN stat;
  int image_data_format;

  if (!hasValidHandle())
  {
    return;
  }
  if (image_format == std::string("XI_MONO16"))
  {
    image_data_format = XI_MONO16;
    encoding_ = std::string("mono16");
    bpp_ = 2;
  }

  else if (image_format == std::string("XI_RGB24"))
  {
    image_data_format = XI_RGB24;
    encoding_ = std::string("bgr8");
    bpp_ = 3;
  }

  else if (image_format == std::string("XI_RGB32"))
  {
    image_data_format = XI_RGB32;
    encoding_ = std::string("bgr16");
    bpp_ = 3;
  }

  else if (image_format == std::string("XI_RGB_PLANAR"))
  {
    image_data_format = XI_MONO8;
    std::cout << "This is unsupported in ROS default to XI_MONO8" << std::endl;
    bpp_ = 1;
  }

  else if (image_format == std::string("XI_RAW8"))
  {
    image_data_format = XI_RAW8;
    encoding_ = std::string("mono8");
    bpp_ = 1;
  }

  else if (image_format == std::string("XI_RAW16"))
  {
    image_data_format = XI_RAW16;
    encoding_ = std::string("mono16");
    bpp_ = 2;
  }

  else
  {
    image_data_format = XI_MONO8;
    encoding_ = std::string("mono8");
    bpp_ = 1;
  }

  stat = xiSetParamInt(xiH_, XI_PRM_IMAGE_DATA_FORMAT, image_data_format);
  errorHandling(stat, "image_format");    // if we cannot set the format then there is something wrong we should probably quit then
  image_data_format_ = image_data_format;
}
