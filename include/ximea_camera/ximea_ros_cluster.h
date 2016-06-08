/******************************************************************************

Copyright 2015  Abdelhamid El-Bably (University of Waterloo)
                      [ahelbably@uwaterloo.ca]
                Arun Das (University of Waterloo)
                      [adas@uwaterloo.ca]
                Michael Tribou (University of Waterloo)
                      [mjtribou@uwaterloo.ca]

All rights reserved.

********************************************************************************/
#ifndef XIMEA_CAMERA_XIMEA_ROS_CLUSTER_H
#define XIMEA_CAMERA_XIMEA_ROS_CLUSTER_H

#include <ximea_camera/ximea_ros_driver.h>
#include <boost/thread.hpp>
#include <string>
#include <vector>
#include <ximea_camera/ximeaConfig.h>

class ximea_ros_cluster
{
public:
  explicit ximea_ros_cluster(int num_cams);
  explicit ximea_ros_cluster(std::vector < std::string > filenames);
  void add_camera(ximea_ros_driver xd);
  void remove_camera(int serial_no);

  // cluster functions
  void clusterInit();
  void clusterAcquire();
  void clusterPublishImages();
  void clusterPublishCamInfo();
  void clusterPublishImageAndCamInfo();
  void clusterEnd();
  bool isDeviceOpen()
  {
    return devices_open_;
  }
  
  // individual camera functions (encapsulated for thread security)
  void setExposure(int serial_no, int time);
  void setGain(int serial_no, float db);
  void setImageDataFormat(int serial_no, std::string s);
  void setROI(int serial_no, int l, int t, int w, int h);
  void dynamicReconfigureCallback(ximea_camera::ximeaConfig &config, uint32_t level);
  void dumpDynamicConfiguration();
  int getExposure(int serial_no){int idx = getCameraIndex(serial_no); if (idx != -1) return cams_[idx].getExposure();}
private:
  std::vector<ximea_ros_driver> cams_;
  std::vector<boost::thread*> threads_;
  bool devices_open_;
  int num_cams_;
  int getCameraIndex(int serial_no);  // this is private so that no one tries to be smart and open/close our cameras externally, in which case we cannot manage
  const int USB_BUS_SAFETY_MARGIN;
  const int USB3_BANDWIDTH;
  bool fixed_init_;
  bool dynamic_reconfigure_modified_;
  //this is a workaround hack to enable multiple cameras. It is not the ideal solution but the fastest that can be configured
  int configEval(const ximea_camera::ximeaConfig & config, int idx, const std::string  & to_eval){
    if (to_eval == "exposure"){
      switch(idx){
      case(0): return config.exposure1;
      case(1): return config.exposure2;
      case(2): return config.exposure3;
      case(3): return config.exposure4;
      default: return -1;
      }
    }
   if (to_eval == "gain"){
      switch(idx){
      case(0): return config.gain1;
      case(1): return config.gain2;
      case(2): return config.gain3;
      case(3): return config.gain4;
      }
   }
   if (to_eval == "rectLeft"){
      switch(idx){
      case(0): return config.rectLeft1;
      case(1): return config.rectLeft2;
      case(2): return config.rectLeft3;
      case(3): return config.rectLeft4;
      }
   }
   if (to_eval == "rectTop"){
      switch(idx){
      case(0): return config.rectTop1;
      case(1): return config.rectTop2;
      case(2): return config.rectTop3;
      case(3): return config.rectTop4;
      }
   }
   if (to_eval == "rectWidth"){
      switch(idx){
      case(0): return config.rectWidth1;
      case(1): return config.rectWidth2;
      case(2): return config.rectWidth3;
      case(3): return config.rectWidth4;
      }
   }
   if (to_eval == "rectHeight"){
      switch(idx){
      case(0): return config.rectHeight1;
      case(1): return config.rectHeight2;
      case(2): return config.rectHeight3;
      case(3): return config.rectHeight4;
      }
   }
  }
};

#endif  // XIMEA_CAMERA_XIMEA_ROS_CLUSTER_H
