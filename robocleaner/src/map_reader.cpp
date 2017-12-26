#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include <sstream>
#include <string>
#include <iostream>
#include <cstdlib>
#include <cmath>

nav_msgs::OccupancyGrid grid;
bool hasGrid = false;



double yaw_to_deg(double x){
	if(x>0)	
	return abs(x*180/M_PI);
	return 360 - abs(x*180/M_PI);
}

void callback1(const nav_msgs::OccupancyGrid& msg)
{
	grid = msg;
	hasGrid = true;
//	std::cout<<"{"<< msg.info.origin.position.x <<", "<< msg.info.origin.position.y <<"}" <<std::endl;
//	std::cout<<"{"<< msg.info.width <<", "<< msg.info.height <<"}" <<std::endl;

}

void callback2(const nav_msgs::Odometry& msg)
{
	if(!hasGrid)
		return;
	double x = msg.pose.pose.position.x;
	double y = msg.pose.pose.position.y;
	
	x/=grid.info.resolution;
	y/=grid.info.resolution;

	int x_cell = grid.info.origin.position.x + x;
	int y_cell = grid.info.origin.position.y + y;
	tf::Quaternion q(msg.pose.pose.orientation.x,
			msg.pose.pose.orientation.y,
			msg.pose.pose.orientation.z,
			msg.pose.pose.orientation.w			
			);

	std::cout<<"{"<< (tf::getYaw(q)) <<"}" <<std::endl;		
	std::cout<<"{"<< yaw_to_deg(tf::getYaw(q)) <<"}" <<std::endl;		
	std::cout<<"{"<< x_cell <<", "<< y_cell <<"}" <<std::endl;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_reader");
  ros::NodeHandle n;

  ros::Subscriber sub1 = n.subscribe("map", 1, callback1);
  ros::Subscriber sub2 = n.subscribe("odom", 1, callback2);
  ros::spin();

  return 0;
}
