#include "ros/ros.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include "lla2enu/ComputeDistance.h"
#include "lla2enu/CustomMsg.h"
#include <cmath>
#include <dynamic_reconfigure/server.h>
#include <lla2enu/parametersConfig.h>

ros::ServiceClient client;
ros::Publisher custom_msg_pub;
double Crash_threshold;
double Unsafe_threshold;

void callback(const nav_msgs::Odometry::ConstPtr& msg1, const nav_msgs::Odometry::ConstPtr& msg2)
{
	//ROS_INFO ("Received two messages: (%f,%f,%f) and (%f,%f,%f)", msg1->pose.pose.position.x, msg1->pose.pose.position.y, msg1->pose.pose.position.z, msg2->pose.pose.position.x, msg2->pose.pose.position.y, msg2->pose.pose.position.z);

	lla2enu::ComputeDistance srv;
	srv.request.car_x = msg1->pose.pose.position.x;
	srv.request.car_y = msg1->pose.pose.position.y;
	srv.request.car_z = msg1->pose.pose.position.z;
	srv.request.obs_x = msg2->pose.pose.position.x;
	srv.request.obs_y = msg2->pose.pose.position.y;
	srv.request.obs_z = msg2->pose.pose.position.z;
	if (client.call(srv))
	  {
	    //ROS_INFO("distance: %f", (float)srv.response.distance);
	  }
	  else
	  {
	    ROS_ERROR("Failed to call service compute_distance");
	  }

	lla2enu::CustomMsg msg;
	msg.distance = srv.response.distance;

	if( isnan(msg.distance) ){ msg.status = "NA"; }
	else if (msg.distance <= Crash_threshold){ msg.status = "Crash"; }
	else if (msg.distance > Crash_threshold && msg.distance <= Unsafe_threshold){ msg.status = "Unsafe"; }	
	else if (msg.distance > Unsafe_threshold){ msg.status = "Safe"; }
	
	custom_msg_pub.publish (msg);
}

void callbackConfig(lla2enu::parametersConfig &config, uint32_t level) {
	ROS_INFO("Reconfigure Request: %f %f ", config.Crash_threshold, config.Unsafe_threshold);
	Crash_threshold = config.Crash_threshold;
	Unsafe_threshold = config.Unsafe_threshold;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "subscriber_sync");
	ROS_INFO("starting node client_custom");

	ros::NodeHandle n;
	client = n.serviceClient<lla2enu::ComputeDistance>("compute_distance");
	custom_msg_pub = n.advertise<lla2enu::CustomMsg>("distance_status", 100);

	message_filters::Subscriber<nav_msgs::Odometry> sub1(n, "odom_car", 1);
	message_filters::Subscriber<nav_msgs::Odometry> sub2(n, "odom_obs", 1);

	typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> MySyncPolicy;


	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub1, sub2);
	sync.registerCallback(boost::bind(&callback, _1, _2));

	dynamic_reconfigure::Server<lla2enu::parametersConfig> server;
	dynamic_reconfigure::Server<lla2enu::parametersConfig>::CallbackType f;

	f = boost::bind(&callbackConfig, _1, _2);
	server.setCallback(f);

	ros::spin();

	return 0;
}

