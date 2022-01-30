//ros imports
#include <ros/ros.h>
#include <ros/console.h>
#include "IILFM.hpp"

int main(int argc, char** argv){
	ros::init(argc, argv, "yorktag_node");
	ROS_INFO("beginning YorkTag Node");
	
	IILFM yt(argc, argv);
	
	ros::spin();
	return 0;
}


