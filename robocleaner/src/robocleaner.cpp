#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
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

void rotateLeft(float n){
	geometry_msgs::Twist msg;
	msg.angular.z = rotspeed;
	pub.publish(msg);
	ros::Duration(rotcoeff*n).sleep();
	msg.angular.z = 0.0;
	pub.publish(msg);
		    
}

void rotateRight(float n){
	geometry_msgs::Twist msg;
	msg.angular.z = -rotspeed;
	pub.publish(msg);
	ros::Duration(rotcoeff*n).sleep();
	msg.angular.z = 0.0;
	pub.publish(msg);
}

void rotateBackward(){
	rotateRight(180);
}

void drive(){
    geometry_msgs::Twist msg;
    msg.linear.x = drivespeed;
    pub.publish(msg);
}

void callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
//    std::cout<<"{"<< msg->range_min <<"}" <<std::endl;
    bool left = false;
    bool front = false;
    bool right = false;

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
	rotateBackward();
	drive();
	return;
    }
    if(left){
	std::cout<<"OOoops. Problems on the left. Go right" <<std::endl;
	rotateRight(90.0 + std::rand()%90);
	drive();
	return;
    }
    if(right){
	std::cout<<"OOoops. Problems on the right. Go left" <<std::endl;
	rotateLeft(90.0 + std::rand()%90);
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
  ros::spin();

  return 0;
}
