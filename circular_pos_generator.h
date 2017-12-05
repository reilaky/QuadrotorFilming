#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <cmath>

#define FOCALLENGTH 500
#define PI 3.14159265359
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

double solveCosine(geometry_msgs::Point p1, geometry_msgs::Point p2, geometry_msgs::Point pos)
{
	double p1p2 = solveDistance(p1, p2);
	double p1pos = solveDistance(p1, pos);
	double dotProduct = (pos.x - p1.x) * (p2.x - p1.x) + (pos.y - p1.y) * (p2.y - p1.y) + (pos.z - p1.z) * (p2.z - p1.z);
	double cosVal = dotProduct / (p1p2 * p1pos);
	return cosVal; 
}


double YCoordinates(double x,double y,double k,double x0)
{
    return k * x0 - k * x + y;
}

void solveCenterPointOfCircle(geometry_msgs::Point p1, geometry_msgs::Point p2, double radius, geometry_msgs::Point& centerPoint)
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
		
    centerPoint = center2;

}

void calCircleParameters(geometry_msgs::Point p1, geometry_msgs::Point p2, point2D pp1, point2D pp2, double& radius, geometry_msgs::Point& centerPoint)
{
	double angle = solveAngle(pp1, pp2);
	double distance = solveDistance(p1, p2);
	radius = (distance / 2) / sin(angle);
	centerPoint.z = p1.z;
	solveCenterPointOfCircle(p1, p2, radius, centerPoint);
	/*ROS_INFO("angle: %f", angle);
	ROS_INFO("radius: %f", radius);
	ROS_INFO("distance: %f", distance);
	ROS_INFO("centerPoint: (%f, %f, %f)", centerPoint.x, centerPoint.y, centerPoint.z);
	*/
}

void solveStartAndEndPoint(double radius, geometry_msgs::Point centerPoint, geometry_msgs::Point p1, geometry_msgs::Point p2, geometry_msgs::Point &start, geometry_msgs::Point &end, double &angle1, double &angle2)
{
	// ROS_INFO("centerPoint (%f, %f, %f)\n", centerPoint.x, centerPoint.y, centerPoint.z);
	// ROS_INFO("p1 (%f, %f, %f)   p2 (%f, %f, %f)", p1.x, p1.y, p1.z, p2.x, p2.y, p2.z);
	double k = (p2.y - p1.y) / (p2.x - p1.x);
	// ROS_INFO("k = %f", k);
	if(p1.x == p2.x)
	{
		start.x = centerPoint.x;
		start.y = centerPoint.y - radius;
		end.x = centerPoint.x;
		end.y = centerPoint.y + radius;
	}
	if(k == 0)
	{
		start.x = centerPoint.x - radius;
		start.y = centerPoint.y;
		end.x =  centerPoint.x + radius;
		end.y = centerPoint.y;
	}
	else
	{
		double offset = centerPoint.y - k * centerPoint.x;
		// y = kx + b
		// distance(start, centerPoinrt) = r;
		double a = (k * k) + 1;
		double b = 2 * (k * (offset - centerPoint.y) - centerPoint.x);
		double c = centerPoint.x * centerPoint.x + (offset - centerPoint.y) * (offset - centerPoint.y) - (radius * radius);
		start.x = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
		start.y = k * start.x + offset;
		end.x = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
		end.y =  k * end.x + offset;		
	}
	// y = centerPoint.y + radius * cos(angle);
	double sinv = (start.x - centerPoint.x) / radius;
	double cosv = (start.y - centerPoint.y) / radius;
	// ROS_INFO("sin1 = %f cos1 = %f ", sinv, cosv);
	if(sinv >= 0 && cosv >= 0) // belong to 1st area
		angle1 = asin(sinv);
	else if(sinv > 0 && cosv <= 0) // belong to 2nd area
		angle1 = acos(cosv);
	else if(sinv < 0 && cosv < 0)
		angle1 = PI - asin(sinv);
	else if(sinv < 0 && cosv > 0)
		angle1 = 2 * PI + asin(sinv);
	
	sinv = (end.x - centerPoint.x) / radius;
	cosv = (end.y - centerPoint.y) / radius;
	// ROS_INFO("sin2 = %f cos2 = %f ", sinv, cosv);
	if(sinv >= 0 && cosv >= 0) // belong to 1st area
		angle2 = asin(sinv);
	else if(sinv > 0 && cosv < 0) // belong to 2nd area
		angle2 = acos(cosv);
	else if(sinv < 0 && cosv <= 0)
		angle2 = PI - asin(sinv);
	else if(sinv < 0 && cosv > 0)
		angle2 = 2 * PI + asin(sinv);
	// ROS_INFO("angle1 = %f, angle2 = %f", angle1, angle2);
	// ROS_INFO("angle1: %f, angle2: %f", angle1, angle2);
}


void generateSerialPoints(int num, double radius, geometry_msgs::Point centerPoint, geometry_msgs::Point p1, geometry_msgs::Point p2, std::vector<geometry_msgs::Point>& serialKiller)
{
	// circle formula: (x - cp.x)^2 + (y - cp.y)^2 = r^2;
	// generator random amount of points on the circle: (at least 3 point)
	serialKiller.clear();
	geometry_msgs::Point temp, start, end;
	start.z = end.z = temp.z = p1.z;
	double angle1, angle2;
	// only draw half circle as traj
	solveStartAndEndPoint(radius, centerPoint, p1, p2, start, end, angle1, angle2);
	temp.x = start.x;
	temp.y = start.y;
	serialKiller.push_back(temp);
	bool arc = true;
	for(int i = 1; i <= num; i++)
	{
			temp.x = centerPoint.x + radius * sin(angle1 + (PI / num * i)); 
			temp.y = centerPoint.y + radius * cos(angle1 + (PI / num * i));
			if(acos(solveCosine(temp, p1, p2)) > PI / 2)
			{
				ROS_INFO("arc = %f", acos(solveCosine(temp, p1, p2)));
				arc = false;
				break;
			}
			serialKiller.push_back(temp);
	}
	if(!arc)
	{
		serialKiller.clear();
		for(int i = 1; i <= num; i++)
		{
			temp.x = centerPoint.x + radius * sin(angle1 - (PI / num * i)); 
			temp.y = centerPoint.y + radius * cos(angle1 - (PI / num * i));
			serialKiller.push_back(temp);
		}
	}
}
