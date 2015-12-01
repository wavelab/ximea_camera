/******************************************************************************

Copyright 2015  Arun Das (University of Waterloo) 
                      [adas@uwaterloo.ca]
                Abdelhamid El-Bably (University of Waterloo)
                      [ahelbably@uwaterloo.ca]
                Michael Tribou (University of Waterloo)
  	                  [mjtribou@uwaterloo.ca]

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************/

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
