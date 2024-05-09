#include "rclcpp/rclcpp.hpp"
#include "my_libas_cam/lbas_cam.h"
#include <sensor_msgs/image_encodings.hpp>
#include <sstream>
#include <std_srvs/srv/empty.h>
#include <iostream>

int main(int argc, char **argv)
{
	std::string camera_name;
	float camera_rate=0.0;
	lbas_cam::LBASCam cam_;
	sensor_msgs::msg::Image			image_msg;
	sensor_msgs::msg::CameraInfo 	cam_msg;
	rclcpp::init(argc, argv);


	rclcpp::NodeOptions lbas_camera;

	auto node = std::make_shared<rclcpp::Node>("my_node");
	image_transport::Publisher image_pub;

    image_transport::ImageTransport it(node); 
	//cam_.Cam_info(lbas_camera,node);
	cam_.Cam_info();
	if(!cam_.Cam_init())

	{
		return false;
	}
	camera_name=cam_.Cam_GetID();	
	image_transport::CameraPublisher pub = it.advertiseCamera("/"+camera_name+"/image_raw", 1000); 
	//topic·¢²¼ÆµÂÊ
	//-------------------10hz-----------------//
	camera_rate=cam_.Cam_FrameRate();
	rclcpp::Rate rate((int)camera_rate);
	//	ROS_INFO("Start to Grab,Refresh:%d!",(int)camera_rate);
	while (rclcpp::ok())
	{
		if(!cam_.GrabImage(&image_msg,&cam_msg))
		{
			std::cout<<"continue"<<std::endl;
			continue;
		}
		pub.publish(image_msg, cam_msg); 
		std::cout<<"publish"<<std::endl;
		rate.sleep();
		
	 
	}

	return true;
}