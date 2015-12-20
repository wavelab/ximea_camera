/******************************************************************************

Copyright 2015  Abdelhamid El-Bably (University of Waterloo)
                      [ahelbably@uwaterloo.ca]
                Arun Das (University of Waterloo)
                      [adas@uwaterloo.ca]
                Michael Tribou (University of Waterloo)
                      [mjtribou@uwaterloo.ca]

All rights reserved.

********************************************************************************/

#include <ximea_camera/ximea_ros_cluster.h>
#include <string>
#include <vector>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "ximea");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  int frame_rate_;
  std::vector<std::string> file_names;

  pnh.param<int>("frame_rate", frame_rate_, 100);
  pnh.getParam("camera_param_file_paths", file_names);
  ros::Rate loop(frame_rate_);

  // check size of camera file
  if (file_names.size() == 0)
  {
    ROS_ERROR("ximea_driver: No camera files name specified. Please set 'camera_param_file_paths' parameter to camera yaml file locations in launch file");
    return 0;
  }
  else
  {
    for (unsigned int i = 0; i < file_names.size(); i++)
    {
      ROS_INFO_STREAM("loading camera parameter file: " << file_names[i] << std::endl);
    }
  }

  ximea_ros_cluster xd(file_names);
  xd.clusterInit();
  while (ros::ok())   // TODO: need to robustify against replugging and cntrlc
  {
    ros::spinOnce();
    xd.clusterAcquire();
    xd.clusterPublishImageAndCamInfo();
    loop.sleep();
  }
  xd.clusterEnd();
  return 1;
}
