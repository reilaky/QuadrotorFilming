#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <cmath>

#define FOCALLENGTH 500
#define WIDTH 640
#define HEIGHT 480

struct point2D
{
	int x, y;
};

// focal length: fx
// camera center: c(0, 0, 0)

// fixed postion of p1 and p2:
// suppose both of them at the same y
// p1.y = p2.y
// thus the circle will at the same y
// p1:(p1.x, p1.y, p1.z)
// p2:(p2.x, p2.y, p2.z)

// on screen plane, fixed position: pp1 and pp2(known)
// pp1: (pp1.x, pp1.y)
// pp2: (pp2.x, pp2.y)

double solveAngle(point2D pp1, point2D pp2)
{
	double angle;
	point2D midPoint;
	midPoint.x = pp1.x;
	midPoint.y = HEIGHT / 2;
	double distance1 = solveDistance(pp1, midPoint);
	double distance2 = solveDistance(midPoint, pp2);
	angle = atan(distance1 / FOCALLENGTH) + atan(distance2 / FOCALLENGTH);
	return angle;
}

double solveDistance(point2D p1, point2D p2)
{
	double distance = sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2));
	return distance;
}

double solveDistance(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
	double distance = sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2) + pow((p1.z - p2.z), 2));
	return distance;
}


void calCircleParameters(geometry_msgs::Point p1, geometry_msgs::Point p2, point2D pp1, point2D pp2, double& radius, geometry_msgs::Point& centerPoint)
{
	double angle = solveAngle(pp1, pp2);
	double p1p2 = solveDistance(p1, p2);
	radius = (p1p2 / 2) / sin(angle);
	ROS_INFO("angle: %f", angle);
	ROS_INFO("radius: %f", radius);
	ROS_INFO("distance: %f", distance);
	ROS_INFO("centerPoint: (%f, %f, %f)", centerPoint.x, centerPoint.y, centerPoint.z);
}


