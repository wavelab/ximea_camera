#include "ximea_ros_cluster.h"

using namespace std;

int main(int argc, char ** argv){
	//string file = "/home/wavelab/WavelabRepo/projects/drivers/ximea_camera/src/cam1.yaml";
	//ximea_driver xd(file);
	ros::init(argc,argv, "ximea");
	ros::NodeHandle nh;		//standard ros nodehanlde
	ros::NodeHandle pnh("~");	//needed for parameter server
	int frame_rate_;
	std::string yaml_file1 = "/home/wavelab3/catkin_ws/src/ximea_camera/src/cam1.yaml";
	std::string yaml_file2 = "/home/wavelab3/catkin_ws/src/ximea_camera/src/cam2.yaml";
	std::string yaml_file3 = "/home/wavelab3/catkin_ws/src/ximea_camera/src/cam3.yaml";
	//std::string yaml_file4 = "/home/adas/catkin_ws/src/ximea_camera/src/cam4.yaml";

  	pnh.param<int>("frame_rate", frame_rate_, 100);
	ros::Rate loop(frame_rate_);
	//ximea_ros_driver xd(nh, "camera1", 0);
	//xd.openDevice();
	//xd.setImageDataFormat("XI_RGB24");
	//xd.setExposure(5000);
	//xd.setROI(172, 274, 940, 480);
	//xd.startAcquisition();
	//ximea_ros_cluster xd(1);

	std::vector<std::string> file_names;

	file_names.push_back(yaml_file1);
	file_names.push_back(yaml_file2);
	file_names.push_back(yaml_file3);
	//file_names.push_back(yaml_file4);

	ximea_ros_cluster xd(file_names);
	std::cout << "we're here" << std::endl;
	xd.clusterInit();
	while (ros::ok()){	//TODO: need to robustify against replugging and cntrlc
		ros::spinOnce();
	//	xd.acquireImage();
		xd.clusterAcquire();
	//	xd.publishImageAndCamInfo();
		xd.clusterPublishImageAndCamInfo();
		loop.sleep();
	}
	xd.clusterEnd();
	return 1;
}
