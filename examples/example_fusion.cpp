#include <ros/ros.h>
#include "cnbiros_fusion/SimpleFusion.hpp"

typedef cnbiros::fusion::SimpleFusion SimpleFusion;

int main(int argc, char** argv) {

	std::string topic = "/source_pointcloud";

	ros::init(argc, argv, "fusion");
	ros::NodeHandle node;
	ros::Rate r(10);

	SimpleFusion fusion(&node, "fusion");

	fusion.AddSource("/infrared", SimpleFusion::AsPointCloud); 
	fusion.AddSource("/camera/scan", SimpleFusion::AsLaserScan); 

	fusion.Start();

	return 0;

}
