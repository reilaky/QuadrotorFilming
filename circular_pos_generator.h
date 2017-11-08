#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <cmath>

#define FOCALLENGTH 500

struct point2D
{
	int x, y;
};

// focal length: fx
// camera center: c (0, 0, 0)

// fixed postion of p1 and p2:
// suppose both of them at the same height
// p1.z = p2.z
// thus the circle will at the same height
// p1:(p1.x, p1.y, p1.z)
// p2:(p2.x, p2.y, p2.z)

// on screen plane, fixed position: pp1 and pp2(known)
// pp1: (pp1.x, pp1.y)
// pp2: (pp2.x, pp2.y)

double solveAngle(point2D pp1, point2D pp2)
{
	double distance = sqrt(pow((pp1.x - pp2.x), 2) + pow((pp1.y - pp2.y), 2)) / 2;
	double angle = atan(distance / FOCALLENGTH) * 2;
	return angle;
}

double solveDistance(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
	double distance = sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2) + pow((p1.z - p2.z), 2));
	return distance;
}


double YCoordinates(double x,double y, double k, double x0)
{
    return k * x0 - k * x + y;
}

void solveCenterPointOfCircle(geometry_msgs::Point p1, geometry_msgs::Point p2, double radius, vector<geometry_msgs::Point>& centerPoint)
{

	// p1, p2 and circle are in the same plane(3d to 2d)
	// circle formula: (x - cp.x)^2 + (y - cp.y)^2 = radius^2
	double k = 0.0, k_vertical = 0.0;
    double mid_x = 0.0, mid_y = 0.0;
    double a = 0.0, b = 0.0, c = 0.0;
    geometry_msgs::Point center1, center2;
    k = (p2.y - p1.y) / (p2.x - p1.x);
    if(k == 0)
    {
        center1.x = (p1.x + p2.x) / 2.0;
        center2.x = (p1.x + p2.x) / 2.0;
        center1.y = p1.y + sqrt(radius * radius -(p1.x - p2.x) * (p1.x - p2.x) / 4.0);
        center2.y = p2.y - sqrt(radius * radius -(p1.x - p2.x) * (p1.x - p2.x) / 4.0);
    }
    else
    {
        k_vertical = -1.0 / k;
        mid_x = (p1.x + p2.x) / 2.0;
        mid_y = (p1.y + p2.y) / 2.0;
        a = 1.0 + k_vertical * k_vertical;
        b = -2 * mid_x - k_vertical * k_vertical * (p1.x + p2.x);
        c = mid_x * mid_x + k_vertical * k_vertical * (p1.x + p2.x) * (p1.x + p2.x) / 4.0 - 
            (radius * radius - ((mid_x - p1.x) * (mid_x - p1.x) + (mid_y - p1.y) * (mid_y - p1.y)));
        
        center1.x = (-1.0 * b + sqrt(b * b - 4 * a * c)) / (2 * a);
        center2.x = (-1.0 * b - sqrt(b * b - 4 * a * c)) / (2 * a);
        center1.y = YCoordinates(mid_x, mid_y, k_vertical, center1.x);
        center2.y = YCoordinates(mid_x, mid_y, k_vertical, center2.x);
    }
		center1.z = p1.z;
		center2.z = p1.z;
		
    centerPoint.push_back(center2);
    centerPoint.push_back(center1);

}


void calCircleParameters(geometry_msgs::Point p1, geometry_msgs::Point p2, point2D pp1, point2D pp2, double& radius, vector<geometry_msgs::Point>& centerPoint)
{
	double angle = solveAngle(pp1, pp2);
	double distance = solveDistance(p1, p2);
	radius = (distance / 2) / sin(angle);
	solveCenterPointOfCircle(p1, p2, radius, centerPoint);
	ROS_INFO("angle: %f", angle);
	ROS_INFO("radius: %f", radius);
	ROS_INFO("distance: %f", distance);
	ROS_INFO("centerPoint: (%f, %f, %f)", centerPoint.x, centerPoint.y, centerPoint.z);
}


bool betweenP1andP2(geometry_msgs::Point p)
{
	double maxX, minX, maxY, minY;
	maxX = max(p1.x, p2.x);
	minX = min(p1.x, p2.x);
	maxY = max(p1.y, p2.y);
	maxY = min(p1.y, p2.y);
	if(p.x >= minX && p.x <= maxX && p.y >= minY && p.y <= maxY)
		return true;
	return false;
}

void generateSerialPoints(int num, double radius, std::vector<geometry_msgs::Point> centerPoint, std::vector<geometry_msgs::Point>& serialKiller)
{
	// circle formula: (x - cp.x)^2 + (y - cp.y)^2 = r^2;
	// generator random amount of points on the circle: (at least 3 point)
	geometry_msgs::Point temp;
	double stride =  (2 * radius) / num;
	
	for(int i = 0; i < centerPoint.size(); i++)
	{
		temp.x = -radius + centerPoint[i].x;
		temp.z = centerPoint[i].z;
		for(int i = 0; i < num; i++)
		{
			double distance = radius * radius - (temp.x - centerPoint[i].x) * (temp.x - centerPoint[i].x);
			if(distance >= 0 && !betweenP1andP2(temp))
			{	
				temp.y = sqrt(distance) + centerPoint[i].y;
				serialKiller.push_back(temp);
				ROS_INFO("serial points: (%f, %f, %f)", temp.x, temp.y, temp.z);
				temp.x += stride;
			}
			
			
		}
	}
}




