#pragma once
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

class LaserCallback
{
	public: 
		float minimal = 10;
		LaserCallback() = default;
		void callback(const sensor_msgs::LaserScan::ConstPtr& msg);
		bool motionMode();
};