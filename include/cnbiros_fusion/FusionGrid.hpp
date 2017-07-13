#ifndef CNBIROS_FUSION_GRID_HPP
#define CNBIROS_FUSION_GRID_HPP

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point32.h>

namespace cnbiros {
	namespace fusion {

class FusionGrid : public grid_map::GridMap {

	public:
		FusionGrid(const std::string& fusion_layer);
		FusionGrid(const std::string& fusion_layer, const float x, const float y, const float r);
		~FusionGrid(void);

		void SetGeometry(const float x, const float y, const float r);
		void SetFrame(const std::string& frame);
		std::string GetFrame(void);
		
		bool Add(const std::string& layer, const double value=0.0f);
		bool Remove(const std::string& layer);
		bool Exists(const std::string& layer);
		FusionGrid::Matrix& Get(void);
		const FusionGrid::Matrix& Get(void) const;
		FusionGrid::Matrix& Get(const std::string& layer);
		FusionGrid::Matrix& Get(const std::string& layer) const;
		
		void Reset(float value=0.0f);
		void Reset(const std::string& layer, float value = 0.0f);
		
		bool ReplaceNaN(float value = 0.0f);
		bool ReplaceNaN(const std::string& layer, float value = 0.0f);
		
		bool Sum(void);
		bool Sum(const std::vector<std::string>& layers);

		bool SetMin(float minimum, const std::string& layer = "");
		bool SetMax(float maximum, const std::string& layer = "");
		bool SetMinMax(float minimum, float maximum, const std::string& layer = "");


		void Update(const std::string& layer, sensor_msgs::LaserScan& msg, float radius = 0.0f);
		void Update(const std::string& layer, sensor_msgs::PointCloud& msg);
		void Update(const std::string& layer, geometry_msgs::Point32& msg);

		grid_map_msgs::GridMap ToMessage(void);

	private:
		std::string fusion_layer_;
};

	}
}


#endif
