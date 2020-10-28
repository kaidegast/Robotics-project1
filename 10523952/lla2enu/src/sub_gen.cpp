#include "ros/ros.h"
#include <string>
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <limits>


class pub_sub
{

ros::Time current_time;
float latitude_init;
float longitude_init;
float h0;
std::string vehicle_name;

private:
ros::NodeHandle n; 

ros::Subscriber sub;
tf::TransformBroadcaster br;
ros::Publisher odom_pub;

public:
	pub_sub(){
	n.getParam("/latitude_init", latitude_init);
	n.getParam("/longitude_init", longitude_init);
	n.getParam("/h0", h0);
	ros::param::get("~vehicle_name", vehicle_name);

	ROS_INFO("starting node %s", vehicle_name.c_str());
	ROS_INFO("STARTING POSITION: [%f, %f, %f]", latitude_init, longitude_init, h0);
	
	sub = n.subscribe("vehicle_input_topic", 1000, &pub_sub::chatterCallback, this);
	odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1);
}
void chatterCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){

	float  xEast;
	float  yNorth;
	float  zUp;
	
	if(msg->status.status != 0){
	// fixed values
	double a = 6378137;
	double b = 6356752.3142;
	double f = (a - b) / a;
	double e_sq = f * (2-f);
	float deg_to_rad = 0.0174533;

	// input data from msg
	float latitude = msg->latitude;
	float longitude = msg->longitude;
	float h = msg->altitude;

	//lla to ecef
	float lamb = deg_to_rad*(latitude);
	float phi = deg_to_rad*(longitude);
	float s = sin(lamb);
	float N = a / sqrt(1 - e_sq * s * s);

	float sin_lambda = sin(lamb);
	float  cos_lambda = cos(lamb);
	float  sin_phi = sin(phi);
	float  cos_phi = cos(phi);

	float  x = (h + N) * cos_lambda * cos_phi;
	float  y = (h + N) * cos_lambda * sin_phi;
	float  z = (h + (1 - e_sq) * N) * sin_lambda;

	// ecef to enu

	lamb = deg_to_rad*(latitude_init);
	phi = deg_to_rad*(longitude_init);
	s = sin(lamb);
	N = a / sqrt(1 - e_sq * s * s);

	sin_lambda = sin(lamb);
	cos_lambda = cos(lamb);
	sin_phi = sin(phi);
	cos_phi = cos(phi);

	float  x0 = (h0 + N) * cos_lambda * cos_phi;
	float  y0 = (h0 + N) * cos_lambda * sin_phi;
	float  z0 = (h0 + (1 - e_sq) * N) * sin_lambda;

	float xd = x - x0;
	float  yd = y - y0;
	float  zd = z - z0;

	xEast = -sin_phi * xd + cos_phi * yd;
	yNorth = -cos_phi * sin_lambda * xd - sin_lambda * sin_phi * yd + cos_lambda * zd;
	zUp = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd;

	//First we publish the transform over tf	
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(xEast, yNorth, zUp) );
	tf::Quaternion q;
	q.setRPY(0, 0, 0);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", vehicle_name));	

	}
	else
	{
	xEast = std::numeric_limits<float>::quiet_NaN();
	yNorth = std::numeric_limits<float>::quiet_NaN();
	zUp = std::numeric_limits<float>::quiet_NaN();
	}
	
	//next we'll publish the odometry message
	nav_msgs::Odometry odom;
	odom.header.stamp = ros::Time::now();
	odom.header.frame_id = vehicle_name;
	odom.child_frame_id = "world";

	odom.pose.pose.position.x = xEast;
	odom.pose.pose.position.y = yNorth;
	odom.pose.pose.position.z = zUp;
	odom_pub.publish(odom);
}
};

int main(int argc, char **argv)
{
 	ros::init(argc, argv, "subscribe_and_publish");
 	pub_sub my_pub_sub;
 	ros::spin();
 	return 0;
}
