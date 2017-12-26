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

double threshold;
double rotcoeff;
double rotspeed;
double drivespeed;

ros::NodeHandle *nh;
ros::Publisher pub;

nav_msgs::OccupancyGrid grid;
int x_cell = 0;
int y_cell = 0;
double yaw = 0;
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
	std::cout<<grid.data.size()<<std::endl;

}

void callback2(const nav_msgs::Odometry& msg)
{
	if(!hasGrid)
		return;
	double x = msg.pose.pose.position.x;
	double y = msg.pose.pose.position.y;
	
	x/=grid.info.resolution;
	y/=grid.info.resolution;

	x_cell = grid.info.origin.position.x + x + 192;
	y_cell = grid.info.origin.position.y + y + 192;
	tf::Quaternion q(msg.pose.pose.orientation.x,
			msg.pose.pose.orientation.y,
			msg.pose.pose.orientation.z,
			msg.pose.pose.orientation.w			
			);
	yaw = yaw_to_deg(tf::getYaw(q));
	//std::cout<<"{"<< (tf::getYaw(q)) <<"}" <<std::endl;		
	//std::cout<<"{"<< yaw_to_deg(tf::getYaw(q)) <<"}" <<std::endl;		
	std::cout<<"{"<< x_cell <<", "<< y_cell <<"}" <<std::endl;
}

bool isUnknwn(float n){
	n = n/180*M_PI;
	int y = sin(n)*10;
	int x = cos(n)*10;
	std::cout<<"before grid check: "<<(y_cell+y)<<" "<<(y_cell+y)*grid.info.width+(x_cell+x)<<std::endl;
	if(grid.data[(y_cell+y)*grid.info.width+(x_cell+x)] == -1){
	std::cout<<"after grid check"<<std::endl;
	return true;
	}
	std::cout<<"after grid check"<<std::endl;
	return false;	
}

void rotateLeft(int n){
	for(int i=180; i<270; i+=5){
		if(isUnknwn((n+i)%360)){
			n = (n+i)%360;
			break;
		}
	}
	geometry_msgs::Twist msg;
	msg.angular.z = rotspeed;
	pub.publish(msg);
	ros::Duration(rotcoeff*n).sleep();
	msg.angular.z = 0.0;
	pub.publish(msg);
		    
}

void rotateRight(int n){
	for(int i=45; i<135; i+=5){
		if(isUnknwn((n+i)%360)){
			n = (n+i)%360;
			break;
		}
	}
	geometry_msgs::Twist msg;
	msg.angular.z = -rotspeed;
	pub.publish(msg);
	ros::Duration(rotcoeff*n).sleep();
	msg.angular.z = 0.0;
	pub.publish(msg);
}

void rotateBackward(float n){
	geometry_msgs::Twist msg;
	msg.angular.z = -rotspeed;
	pub.publish(msg);
	ros::Duration(rotcoeff*n).sleep();
	msg.angular.z = 0.0;
	pub.publish(msg);
}

void drive(){
    geometry_msgs::Twist msg;
    msg.linear.x = drivespeed;
    pub.publish(msg);
}

void callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	if(!hasGrid)
		return;
//    std::cout<<"{"<< msg->range_min <<"}" <<std::endl;
    bool left = false;
    bool front = false;
    bool right = false;

    double world_angle = yaw;

    for(int i=0;i<10;i++)
	if(msg->ranges[i] < threshold){
		front = true;
		break;
	}
	
    for(int i=10;i<45;i++)
	if(msg->ranges[i] < threshold){
		left = true;
		break;
	}
    for(int i=314;i<350;i++)
	if(msg->ranges[i] < threshold){
		right = true;
		break;
	}

    for(int i=350;i<360;i++)
	if(msg->ranges[i] < threshold){
		front = true;
		break;
	}


    //std::cout<<"{"<< left <<", "<< front << ", "<< right<<"}" <<std::endl;
    if(front){
	std::cout<<"OOoops. Problems on the front. Go back" <<std::endl;
	rotateBackward(world_angle);
	drive();
	return;
    }
    if(left){
	world_angle = (int)(world_angle+25)%360;
	std::cout<<"OOoops. Problems on the left. Go right" <<std::endl;
	rotateRight(world_angle);
	drive();
	return;
    }
    if(right){
	world_angle = abs(world_angle-25);
	std::cout<<"OOoops. Problems on the right. Go left" <<std::endl;
	rotateLeft(world_angle);
	drive();
	return;
    }
    
    std::cout<<"Go to the future!" <<std::endl;
    drive();

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "lzrsnecho");
  ros::NodeHandle n;
  ros::NodeHandle n1("~");
  n1.getParam("threshold", threshold);
  n1.getParam("rotcoeff", rotcoeff);
  n1.getParam("rotspeed", rotspeed);
  n1.getParam("drivespeed", drivespeed);

  nh = &n;
  pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::Subscriber sub = n.subscribe("scan", 1, callback);
  ros::Subscriber sub1 = n.subscribe("map", 1, callback1);
  ros::Subscriber sub2 = n.subscribe("odom", 1, callback2);
  ros::spin();

  return 0;
}
