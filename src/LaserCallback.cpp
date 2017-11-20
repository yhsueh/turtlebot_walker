#include "LaserCallback.hpp"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/LaserScan.h"
#include <math.h> 


void LaserCallback::callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  int lengthArray;
  minimal = 10;
  lengthArray = (sizeof(msg->ranges)/sizeof(msg->ranges[0]));

  for (auto i: msg->ranges){
    if (i < minimal){
      minimal = i;
    }
  }

  std::cout << "minimal value: " << minimal << std::endl;
}


bool LaserCallback::motionMode() {
	    if (minimal < 1 || std::isnan(minimal)) {
      return false;
    }else{
      return true;
    }

  }




