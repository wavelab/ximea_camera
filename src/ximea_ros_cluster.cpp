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
#include <fstream> 

int serial_nos[3] = { 32300651 ,  33300151 , 32301251};
std::string cam_names[3] = {std::string("camera1"), std::string("camera2"), std::string("camera3")};
std::string calib_file_names[3] =
{
  "package://mcptam/calibrations/camera1.yaml",
  "package://mcptam/calibrations/camera2.yaml",
  "package://mcptam/calibrations/camera3.yaml"
};

std::string getCamNameFromYaml(std::string file_name)
{
  std::ifstream fin(file_name.c_str());
  if (fin.fail())
  {
    ROS_ERROR_STREAM("could not open file " << file_name.c_str() << std::endl);
    exit(-1);  // this has to be changed
  }

  YAML::Node doc = YAML::LoadFile(file_name);
  std::string ret;
  ret = doc["cam_name"].as<std::string>();
  return ret;
}

ximea_ros_cluster::ximea_ros_cluster(int num_cams) : USB_BUS_SAFETY_MARGIN(0), USB3_BANDWIDTH(2400)
{
  num_cams_ = num_cams;
  devices_open_ = false;
  for (int i = 0 ; i < num_cams; i ++)
  {
    ros::NodeHandle nh(std::string("/") + cam_names[i]);
    add_camera(ximea_ros_driver(nh, cam_names[i], serial_nos[i], calib_file_names[i]));
  }
  // must limit the cluster usb bandwidth to support > 2 cameras
  xiSetParamInt(0, XI_PRM_AUTO_BANDWIDTH_CALCULATION, XI_OFF);
  fixed_init_ = true;
  dynamic_reconfigure_modified_ = false;
}

ximea_ros_cluster::ximea_ros_cluster(std::vector<std::string> filenames, std::string config_directory) : USB_BUS_SAFETY_MARGIN(0), USB3_BANDWIDTH(2400)
{
  devices_open_ = false;
  for (int i = 0 ; i < filenames.size(); i ++)
  {
    std::string cam_name = getCamNameFromYaml(filenames[i]);
    ros::NodeHandle nh(std::string("/") + cam_name);
    add_camera(ximea_ros_driver(nh, filenames[i]));
  }
  config_directory_ = config_directory;
  // must limit the cluster usb bandwidth to support > 2 cameras
  xiSetParamInt(0, XI_PRM_AUTO_BANDWIDTH_CALCULATION, XI_OFF);
  fixed_init_ = false;
  dynamic_reconfigure_modified_ = false;
}

void ximea_ros_cluster::add_camera(ximea_ros_driver xd)
{
  if (devices_open_)
  {
    clusterEnd();
  }
  cams_.push_back(xd);
  num_cams_++;
  threads_.resize(num_cams_);
  ROS_INFO_STREAM("done camera add");
}

void ximea_ros_cluster::remove_camera(int serial_no)
{
  if (devices_open_)
  {
    clusterEnd();
  }
  for (int i = 0; i < cams_.size(); i++)
  {
    if (serial_no == cams_[i].getSerialNo())
    {
      cams_.erase(cams_.begin() + i);
      delete threads_[i];
      threads_.erase(threads_.begin() + i);
      break;
    }
  }
  num_cams_--;
}

void ximea_ros_cluster::clusterInit()
{
  for (int i = 0; i < cams_.size(); i++)
  {
    ROS_INFO_STREAM("opening device " << cams_[i].getSerialNo());
    cams_[i].openDevice();
    if (fixed_init_)
    {
      cams_[i].setImageDataFormat("XI_MONO8");
      cams_[i].setROI(200, 200, 900, 600);
      cams_[i].setExposure(10000);
    }
    cams_[i].limitBandwidth((USB3_BANDWIDTH) - USB_BUS_SAFETY_MARGIN);
    cams_[i].startAcquisition();
    // TODO: remove this into constructor
  }
  devices_open_ = true;
}

void ximea_ros_cluster::clusterEnd()
{
  for (int i = 0; i < cams_.size(); i  ++)
  {
    cams_[i].stopAcquisition();
    cams_[i].closeDevice();
  }
  devices_open_ = false;
}

// triggered_acquire
void ximea_ros_cluster::clusterAcquire()
{
  for (int i = 0; i < cams_.size(); i  ++)
  {
    threads_[i] = new boost::thread(&ximea_driver::acquireImage, &cams_[i]);
  }
  for (int i = 0; i < cams_.size(); i  ++)
  {
    threads_[i]->join();
    delete threads_[i];
  }
}

void ximea_ros_cluster::clusterPublishImages()
{
  // TODO: might want to think as to how to multithread this
  for (int i = 0; i < cams_.size(); i  ++)
  {
    threads_[i] = new boost::thread(&ximea_ros_driver::publishImage, &cams_[i], ros::Time::now());
  }
  for (int i = 0; i < cams_.size(); i  ++)
  {
    threads_[i]->join();
    delete threads_[i];
  }
}

void ximea_ros_cluster::dynamicReconfigureCallback(ximea_camera::ximeaConfig &config, uint32_t level){
  //this is a hack to get around rqt_reconfigure limitations a better solution would be to get rid of rqt_reconfigure and make a specific ximea_reconfigure gui

  int idx = getCameraIndex(level);

  std::cout << "idx " << idx << std::endl;
  if (idx != -1){
    int config_exposure = configEval(config, idx, "exposure");
    int config_rectLeft = configEval(config, idx, "rectLeft");
    int config_rectTop = configEval(config, idx, "rectTop");
    int config_rectWidth = configEval(config, idx, "rectWidth");
    int config_rectHeight = configEval(config, idx, "rectHeight");
    float config_gain = (float)configEval(config, idx, "gain");
    if (cams_[idx].getExposure() != config_exposure){
     std::cout << "setting exposure "  << std::endl;
     setExposure(level, config_exposure);
    }
    if (cams_[idx].getGain() != config_gain){
     std::cout << "setting Gain "  << std::endl;
     setGain(level, config_gain);
    }
    rect r = cams_[idx].getRect();
    if (r.x != config_rectLeft || r.y != config_rectTop || r.w != config_rectWidth || r.h != config_rectHeight){
      setROI(level, config_rectLeft, config_rectTop, config_rectWidth, config_rectHeight);
    }
    dynamic_reconfigure_modified_ = true;
  }
}

void ximea_ros_cluster::clusterPublishCamInfo()
{
  for (int i = 0 ; i < cams_.size(); i ++)
  {
    cams_[i].publishCamInfo(ros::Time::now());
  }
}

void ximea_ros_cluster::clusterPublishImageAndCamInfo()
{
  ros::Time curr_time = ros::Time::now();
  for (int i = 0; i < cams_.size(); i  ++)
  {
    threads_[i] = new boost::thread(&ximea_ros_driver::publishImage, &cams_[i], curr_time);
  }
  for (int i = 0; i < cams_.size(); i  ++)
  {
    threads_[i]->join();
    delete threads_[i];
  }
  for (int i = 0 ; i < cams_.size(); i ++)
  {
    cams_[i].publishCamInfo(curr_time);
  }
}

int ximea_ros_cluster::getCameraIndex(int serial_no)
{
  for (int i = 0; i < cams_.size(); i++)
  {
    if (serial_no == cams_[i].getSerialNo())
    {
      return i;
    }
  }
  return -1;
}

void ximea_ros_cluster::setExposure(int serial_no, int time)
{
  int idx = getCameraIndex(serial_no) ;
  if (idx != -1)
  {
    cams_[idx].setExposure(time);
  }
}

void ximea_ros_cluster::setGain(int serial_no, float db)
{
  int idx = getCameraIndex(serial_no) ;
  if (idx != -1)
  {
    cams_[idx].setGain(db);
  }
}

void ximea_ros_cluster::setImageDataFormat(int serial_no, std::string s)
{
  int idx = getCameraIndex(serial_no) ;
  if (idx != -1)
  {
    cams_[idx].setImageDataFormat(s);
  }
}

void ximea_ros_cluster::setROI(int serial_no, int l, int t, int w, int h)
{
  int idx = getCameraIndex(serial_no) ;
  if (idx != -1)
  {
    cams_[idx].setROI(l, t, w, h);
  }
}

void ximea_ros_cluster::dumpDynamicConfiguration(){
  if (!dynamic_reconfigure_modified_) return;
  std::ostringstream file_name_stream, os; 
  std::ofstream f;
  std::string directory = config_directory_;
  std::cout << "dumping config params to yaml file in " << config_directory_ << std::endl;
  for (int i = 0; i < cams_.size(); i ++){
    file_name_stream << directory << cams_[i].getCamName() << "_dyn_reconfigure.yaml";
    f.open(file_name_stream.str().c_str());
    if (!f.is_open()) {std::cout << "could not dump file "; continue;}
    std::cout << cams_[i].getExposure() << std::endl;
    os << "serial_no: " << cams_[i].getSerialNo() << std::endl;
    os << "cam_name: \"" << cams_[i].getCamName() << "\"" << std::endl;
    os << "yaml_url: \"" << cams_[i].getYamlURL() << "\"" <<  std::endl;
    rect r = cams_[i].getRect();
    os << "exposure: " << cams_[i].getExposure() << std::endl;
    os << "gain: " << cams_[i].getGain() << std::endl;
    os << "rect_left: " << r.x << std::endl;
    os << "rect_top: " << r.y << std::endl;
    os << "rect_height: " << r.h << std::endl;
    os << "rect_width: " << r.w << std::endl;
    os << "image_data_format: \"" << cams_[i].getImageDataFormat() << "\"" << std::endl;
    f << os.str(); 
    f.close();
  }
}
