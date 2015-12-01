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

#include "ximea_driver.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>


class ximea_ros_driver : public ximea_driver{
	public:
	ximea_ros_driver( const ros::NodeHandle &nh, std::string cam_name, int serial_no , std::string yaml_url);
	ximea_ros_driver( const ros::NodeHandle &nh, std::string file_name);
	//~ximea_ros_driver();
	virtual void setImageDataFormat(std::string s);
	void publishImage(const ros::Time & now);	//since these 2 functions should have the same time stamp we leave it up to the user to specify the timeif it is needed to do one or the other
	void publishCamInfo(const ros::Time &now);
	void publishImageAndCamInfo();
	
	protected:
	ros::NodeHandle pnh_;
	camera_info_manager::CameraInfoManager *cam_info_manager_;
	image_transport::ImageTransport *it_;
	image_transport::Publisher ros_cam_pub_;
	ros::Publisher cam_info_pub_;

	sensor_msgs::Image ros_image_;
	sensor_msgs::CameraInfo cam_info_;
	char * cam_buffer_;
	int cam_buffer_size_;
	int bpp_;		//the next 2 paramaeters are used by the ros_image_transport publisher
	std::string encoding_;
		
	private:
	void common_initialize(const ros::NodeHandle &nh);
};
