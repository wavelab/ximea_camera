#include "ximea_ros_driver.h"
#include <boost/thread.hpp>

class ximea_ros_cluster{
	public:
	//ximea_ros_cluster(int num_cams, std::vector<std::string> config_files); TODO: this is what we actually need will test with the one below
	//ximea_ros_cluster(const ros::NodeHandle & nh, int num_cams);
	ximea_ros_cluster(int num_cams);
	ximea_ros_cluster(std::vector < std::string > filenames);
	//ximea_ros_cluster(const Nodehandle & nh, std::vector<ximea_ros_driver> cams);
	void add_camera(ximea_ros_driver xd);
	void remove_camera(int serial_no);

	//cluster functions
	void clusterInit();
	void clusterAcquire();
	void clusterPublishImages();
	void clusterPublishCamInfo();
	void clusterPublishImageAndCamInfo();
	void clusterEnd();
	bool isDeviceOpen(){return devices_open_;}
	
	//individual camera functions (encapsulated for thread security)
	void setExposure(int serial_no, int time);
	void setImageDataFormat(int serial_no, std::string s);
	void setROI(int serial_no, int l, int t, int w, int h);

	private:
	std::vector<ximea_ros_driver> cams_;
	std::vector<boost::thread*> threads_;
	bool devices_open_;
	int num_cams_;
	int getCameraIndex(int serial_no);	//this is private so that no one tries to be smart and open/close our cameras externally, in which case we cannot manage
	const int USB_BUS_SAFETY_MARGIN;
	const int USB3_BANDWIDTH;
	bool fixed_init_;
};
