#ifndef CNBIROS_FUSION_SIMPLEFUSION_CPP
#define CNBIROS_FUSION_SIMPLEFUSION_CPP

#include "cnbiros_fusion/SimpleFusion.hpp"

namespace cnbiros {
	namespace fusion {

SimpleFusion::SimpleFusion(ros::NodeHandle* node, std::string name) : cnbiros::core::NodeInterface(node, name) {
	this->sources_ = new cnbiros::core::Subscribers(node);
	
	this->rospub_ = node->advertise<grid_map_msgs::GridMap>("/"+name, CNBIROS_CORE_BUFFER_MESSAGES);
	
	this->fusiongrid_ = new FusionGrid(name, CNBIROS_FUSION_GRID_X,
									   		 CNBIROS_FUSION_GRID_Y,
									   		 CNBIROS_FUSION_GRID_R);
}

SimpleFusion::~SimpleFusion(void) {
	delete this->sources_;
	delete this->fusiongrid_;
}

bool SimpleFusion::RemoveSource(const std::string topic) {
	return this->sources_->Remove(topic);
}

bool SimpleFusion::AddSource(const std::string topic, unsigned int type) {

	bool retcode = false;
	switch(type) {
		case SimpleFusion::AsPointCloud:
			ROS_INFO("Topic %s added as source (PointCloud)", topic.c_str());
			retcode = this->sources_->Add<sensor_msgs::PointCloud>(topic, 
					  boost::bind(&SimpleFusion::on_received_pointcloud, this, _1, topic));
			break;
		case SimpleFusion::AsLaserScan:
			ROS_INFO("Topic %s added as source (LaserScan)", topic.c_str());
			retcode = this->sources_->Add<sensor_msgs::LaserScan>(topic, 
					  boost::bind(&SimpleFusion::on_received_laserscan, this, _1, topic));
			break;
		case SimpleFusion::AsPoint:
			ROS_INFO("Topic %s added as source (Point)", topic.c_str());
			retcode = this->sources_->Add<geometry_msgs::Point32>(topic, 
					  boost::bind(&SimpleFusion::on_received_point, this, _1, topic));
			break;
		default:
			break;
	}

	retcode = retcode & this->fusiongrid_->Add(topic);

	return retcode;
}

void SimpleFusion::on_received_pointcloud(const sensor_msgs::PointCloud::ConstPtr& msg, std::string topic) {
	sensor_msgs::PointCloud data = *msg;
	this->fusiongrid_->Reset(topic);
	this->fusiongrid_->Update(topic, data);
}

void SimpleFusion::on_received_laserscan(const sensor_msgs::LaserScan::ConstPtr& msg, std::string topic) {
	sensor_msgs::LaserScan data = *msg;
	this->fusiongrid_->Reset(topic);
	this->fusiongrid_->Update(topic, data);
}

void SimpleFusion::on_received_point(const geometry_msgs::Point32::ConstPtr& msg, std::string topic) {
	geometry_msgs::Point32 data = *msg;
	this->fusiongrid_->Reset(topic);
	this->fusiongrid_->Update(topic, data);
}


void SimpleFusion::onRunning(void) {

	grid_map_msgs::GridMap msg;

	// Reset fusion layer
	this->fusiongrid_->Reset();

	// Sum all other layers
	this->fusiongrid_->Sum();
	
	// Normalize fusion layer
	this->fusiongrid_->SetMax(1.0f);

	// Publish message
	this->rospub_.publish(this->fusiongrid_->ToMessage());
}


	}
}


#endif
