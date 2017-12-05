#include <tf/transform_broadcaster.h>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include "ros/ros.h"
#include "limits.h"

double yaw = 0.0;
double orientation[4] = {0, 0, 0, 1};

double solveDist(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
	double distance = sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2) + pow((p1.z - p2.z), 2));
	return distance;
}

double solveCos(geometry_msgs::Point p1, geometry_msgs::Point p2, geometry_msgs::Point pos)
{
	double p1p2 = solveDist(p1, p2);
	double p1pos = solveDist(p1, pos);
	double dotProduct = (pos.x - p1.x) * (p2.x - p1.x) + (pos.y - p1.y) * (p2.y - p1.y) + (pos.z - p1.z) * (p2.z - p1.z);
	double cosVal = dotProduct / (solveDist(p1, p2) * solveDist(p1, pos));
	return cosVal; 
}

void toQuaternion(double pitch, double roll, double yaw)
{
  // Abbreviations for the various angular functions
	double cy = cos(yaw * 0.5);
	double sy = sin(yaw * 0.5);
	double cr = cos(roll * 0.5);
	double sr = sin(roll * 0.5);
	double cp = cos(pitch * 0.5);
	double sp = sin(pitch * 0.5);

	orientation[3] = cy * cr * cp + sy * sr * sp;
	orientation[0] = cy * sr * cp - sy * cr * sp;
	orientation[1] = cy * cr * sp + sy * sr * cp;
	orientation[2] = sy * cr * cp - cy * sr * sp;
}

void setYaw(geometry_msgs::Point center, geometry_msgs::Point pos)
{
	/*
	// calculate the yaw
	yaw = acos((center.x - pos.x) / sqrt((center.x - pos.x) * (center.x - pos.x) + (center.y - pos.y) * (center.y - pos.y) + (center.z - pos.z) * (center.z - pos.z)));
	if(center.y - pos.y > 0)
	yaw = -yaw;
	*/


     double dx = abs(center.x - pos.x);
	double dy = abs(center.y - pos.y);
	yaw = atan(dy/dx);

     if(center.x > pos.x && center.y < pos.y) yaw = -yaw;
     if(center.x > pos.x && center.y > pos.y) yaw = yaw;
     if(center.x < pos.x && center.y > pos.y) yaw = PI-yaw;
     if(center.x < pos.x && center.y < pos.y) yaw = PI+yaw;


	// set the gimbal control
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(0, 0, 0) );
	tf::Quaternion q;
	q.setRPY(0, -yaw, 0.0);
	tf::Matrix3x3 m(q);
	//m.getRPY(roll, pitch, yaw);
	//q.setRPY(0.0, 0.0, 0.0);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "gimbal", "camera1"));
}

void setWaypoint(geometry_msgs::Point waypoint, nav_msgs::Path &path)
{
	geometry_msgs::PoseStamped pose;
	toQuaternion(0, 0, yaw);
	pose.pose.position.x = waypoint.x;
	pose.pose.position.y = waypoint.y;
	pose.pose.position.z = waypoint.z;
	pose.pose.orientation.x = orientation[0];
	pose.pose.orientation.y = orientation[1];
	pose.pose.orientation.z = orientation[2];
  pose.pose.orientation.w = orientation[3];
  //ROS_INFO("path: (%f, %f, %f)", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
  path.poses.push_back(pose);
}

int findCloestWaypointIndex(std::vector<geometry_msgs::Point> waypoints, geometry_msgs::Point pos)
{
	double minDist = DBL_MAX;
	int minIndex = 0;
	for(int i = 0; i < waypoints.size(); i++)
	{
		double distance = solveDist(waypoints[i], pos);
		//ROS_INFO("distance[%d]: %f",i, distance);
		if(minDist > distance)
		{
			minIndex = i;
			minDist = distance;
		}
	}
	
	if(minIndex == waypoints.size() - 1)
		return minIndex;
	else
	{
		double pp1 = solveCos(waypoints[minIndex], waypoints[minIndex + 1], pos);
		if(pp1 < 0)
			return  minIndex;
		else
			return minIndex + 1; 
	}
	
}

void setPath(std::vector<geometry_msgs::Point> waypoints, int &trigger, nav_msgs::Path &path, geometry_msgs::Point pos)
{
  if(trigger == 0)
		ROS_INFO("Haven't been triggered yet, no path will be sent.");
	else if(trigger == 1)
	{
		//ROS_INFO("Send path to traj planning.");
		// setWaypoint(pos, path);
		int i = findCloestWaypointIndex(waypoints, pos) + 1;
		// ROS_INFO("index: %d", i);
		// for(; i < waypoints.size(); i++)
		for(int j = 0; j < 2; j++)
		{
			if(i + j < waypoints.size())
				setWaypoint(waypoints[i + j], path);
		}
			
  }
}
