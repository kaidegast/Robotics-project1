#include "ros/ros.h"
#include "lla2enu/ComputeDistance.h"
#include <math.h>
#include <cmath>

bool calculateDistance(lla2enu::ComputeDistance::Request &req,
         lla2enu::ComputeDistance::Response &res)
{
	if( isnan(req.car_x) || isnan(req.obs_x) ){
	res.distance = std::numeric_limits<float>::quiet_NaN();
	}
	else
	{
	res.distance = sqrt(pow(req.car_x - req.obs_x, 2) + pow(req.car_y - req.obs_y, 2) + pow(req.car_z - req.obs_z, 2));
	}
	//ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
	//ROS_INFO("sending back response: [%f]", (float)res.distance);
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "compute_distance_server");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("compute_distance", calculateDistance);
	ROS_INFO("Ready to compute distance.");
	ros::spin();

	return 0;
}
