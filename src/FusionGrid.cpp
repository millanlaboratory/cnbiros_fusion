#ifndef CNBIROS_FUSION_GRID_CPP
#define CNBIROS_FUSION_GRID_CPP

#include "cnbiros_fusion/FusionGrid.hpp"

namespace cnbiros {
	namespace fusion {

FusionGrid::FusionGrid(const std::string& fusion_layer) {
	this->fusion_layer_ = fusion_layer;
	this->Add(this->fusion_layer_);
	this->SetFrame("base_link");
}

FusionGrid::FusionGrid(const std::string& fusion_layer, const float x, const float y, const float r) {
	this->fusion_layer_ = fusion_layer;
	this->SetGeometry(x, y, r);
	this->Add(this->fusion_layer_);
	this->SetFrame("base_link");
}

FusionGrid::~FusionGrid(void) {}

void FusionGrid::SetGeometry(const float x, const float y, const float r) {
	this->setGeometry(grid_map::Length(x, y), r);
}

void FusionGrid::SetFrame(const std::string& frame) {
	this->setFrameId(frame);
}

std::string FusionGrid::GetFrame(void) {
	return this->getFrameId();
}

bool FusionGrid::Add(const std::string& layer,  const double value) {

	bool retcod = false;
	if(this->Exists(layer) == false) {
		this->add(layer, value);
		retcod = true;
	}
	return retcod;
}

bool FusionGrid::Remove(const std::string& layer) {
	return this->erase(layer);
}

bool FusionGrid::Exists(const std::string& layer) {
	return this->exists(layer);
}

FusionGrid::Matrix& FusionGrid::Get(void) {
	return this->get(this->fusion_layer_);
}

const FusionGrid::Matrix& FusionGrid::Get(void) const {
	return this->get(this->fusion_layer_);
}

void FusionGrid::Reset(float value) {
	this->Reset(this->fusion_layer_, value);
}

void FusionGrid::Reset(const std::string& layer, float value) {

	if(this->Exists(layer)) {
		this->get(layer).setConstant(value);
	}
}


bool FusionGrid::Sum(void) {
	bool result;
	std::vector<std::string> layers;
	layers = this->getLayers();

	result = this->Sum(layers);
	
	return result;
}

bool FusionGrid::Sum(const std::vector<std::string>& layers) {
	bool result = true;
	std::vector<std::string>::const_iterator it;

	for(it = layers.begin(); it != layers.end(); ++it) {
		if(this->Exists(*it) == true) {
			if((*it).compare(this->fusion_layer_) != 0) {
				this->get(this->fusion_layer_) += this->get(*it);
			}
		}
	}

	return result;
}

bool FusionGrid::ReplaceNaN(float value) {
	return this->ReplaceNaN(this->fusion_layer_, value);
}

bool FusionGrid::ReplaceNaN(const std::string& layer, float value) {

	bool result = false;
	if(this->Exists(layer)) {
		this->get(layer) = (this->get(layer).array().isNaN()).select(0.0f, this->get(layer)); 	
		result = true;
	}

	return result;
}


bool FusionGrid::SetMin(float minimum, const std::string& layer) {
	bool result = false;
	std::string target = layer;

	if(target.empty())
		target = this->fusion_layer_;

	if(this->Exists(target) == true) {
		this->get(target) = (this->get(target).array() < minimum).select(minimum, this->get(target));
		result = true;
	}

	return result;
}

bool FusionGrid::SetMax(float maximum, const std::string& layer) {
	bool result = false;
	std::string target = layer;

	if(target.empty())
		target = this->fusion_layer_;
	
	if(this->Exists(target) == true) {
		this->get(target) = (this->get(target).array() > maximum).select(maximum, this->get(target));
		result = true;
	}

	return result;
}

bool FusionGrid::SetMinMax(float minimum, float maximum, const std::string& layer) {

	std::string target = layer;

	if(target.empty())
		target = this->fusion_layer_;

	return this->SetMin(minimum, layer) & this->SetMax(maximum, layer);
}

void FusionGrid::Update(const std::string& layer, sensor_msgs::LaserScan& msg, float radius) {

	float angle, x, y;
	std::vector<float>::iterator itr;

	angle = msg.angle_min;
	
	for (itr = msg.ranges.begin(); itr != msg.ranges.end(); ++itr) {
					
		// Skip ranges with inf value
		if(std::isinf(*itr) == false) {
			
			// Get cartesian cohordinates -> To be added: position of the kinect
			x = (*itr + radius)*cos(angle);
			y = (*itr + radius)*sin(angle);

			grid_map::Position position(x, y);

			// Skip positions outside the grid range
			if(this->isInside(position)) {
					
				// Fill the grid cell if ranges are between the min/max limits
				if ( ((*itr) > msg.range_min) & ((*itr) < msg.range_max)) {
					this->atPosition(layer, position) = 1.0f;
				} else {
					this->atPosition(layer, position) = 0.0f;
				}

			}
		}
		
		angle += msg.angle_increment;
	}
}

void FusionGrid::Update(const std::string& layer, sensor_msgs::PointCloud& msg) {

	grid_map::Position position;
	grid_map::Index    index;

	for(auto it = msg.points.begin(); it != msg.points.end(); ++it) {
	
		position = grid_map::Position((*it).x, (*it).y);

		if(this->isInside(position)) {
			this->atPosition(layer, position) = 1.0f;
		}
	}
}

void FusionGrid::Update(const std::string& layer, geometry_msgs::Point32& msg) {

	grid_map::Position position;
	position = grid_map::Position(msg.x, msg.y);
	
	if(this->isInside(position)) {
		this->atPosition(layer, position) = 1.0f;
	}
}


grid_map_msgs::GridMap FusionGrid::ToMessage(void) {
	grid_map_msgs::GridMap msg;
	grid_map::GridMapRosConverter::toMessage(*this, msg);
	return msg;
}

	}
}



#endif
