#include <ros.h>
#include <math.h>  

struct point2D
{
	double x, y;
}

const static focalLength = 500;

// screen size: 640 * 480
// suppose:
// focal length: fx
// camera center: c (0, 0, 0)
// screen position: s (0, 0, fx); pixels

// exchanging coordinate needed(between world coord and camera coord)

// fixed postion of p1 and p2:
// suppose both of them at the same height
// p1.z = p2.z
// thus the circle will at the same height
// p1:(p1.x, p1.y, p1.z)
// p2:(p2.x, p2.y, p2.z)

// on screen plane, fixed position: pp1 and pp2(known)
// pp1: (pp1.x, pp1.y)
// pp2: (pp2.x, pp2.y)

// calculate the angle of pp1-c-pp2:
double radius;
geometry_msgs::Point centerPoint;
int num = 30;
double stride = 0.2;
std::vector<geometry_msgs::Point> serialKiller;

calCircleParameters(p1, p2, pp1, pp2, radius, centerPoint);
generateSerialPoints(num, stride,radius, centerPoint, serialKiller);

void calCircleParameters(geometry_msgs::Point p1, geometry_msgs::Point p2, point2D pp1, point2D pp2, double& radius, geometry_msgs::Point& centerPoint)
{
	double angle = solveAngle(pp1, pp2);
	double distance = solveDistance(p1, p2);
	radius = (distance / 2) / sin(angle);
	centerPoint.z = p1.z;
	solveCenterPointOfCircle(p1, p2, radius, centerPoint);
}

double solveAngle(point2D pp1, point2D pp2)
{
	double distance = sqrt(pow((pp1.x - pp2.x), 2) + pow((pp1.y - pp2.y), 2)) / 2;
	return atan(distance / focalLength) * 2;
}

double solveDistance(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
	return sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2) + pow((p1.z - p2.z), 2));
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
        center1.y = p1.y + sqrt(dRadius * dRadius -(p1.x - p2.x) * (p1.x - p2.x) / 4.0);
        center2.y = p2.y - sqrt(dRadius * dRadius -(p1.x - p2.x) * (p1.x - p2.x) / 4.0);
    }
    else
    {
        k_vertical = -1.0 / k;
        mid_x = (p1.x + p2.x) / 2.0;
        mid_y = (p1.y + p2.y) / 2.0;
        a = 1.0 + k_vertical * k_vertical;
        b = -2 * mid_x - k_vertical * k_vertical * (p1.x + p2.x);
        c = mid_x * mid_x + k_vertical * k_vertical * (p1.x + p2.x) * (p1.x + p2.x) / 4.0 - 
            (dRadius * dRadius - ((mid_x - p1.x) * (mid_x - p1.x) + (mid_y - p1.y) * (mid_y - p1.y)));
        
        center1.x = (-1.0 * b + sqrt(b * b - 4 * a * c)) / (2 * a);
        center2.x = (-1.0 * b - sqrt(b * b - 4 * a * c)) / (2 * a);
        center1.y = Y_Coordinates(mid_x, mid_y, k_vertical, center1.x);
        center2.y = Y_Coordinates(mid_x, mid_y, k_vertical, center2.x);
    }

    centerPoint = center1;

}

double YCoordinates(double x,double y,double k,double x0)
{
    return k * x0 - k * x + y;
}

void generateSerialPoints(int num, double stride, double& radius, geometry_msgs::Point centerPoint, std::vector<geometry_msgs::Point> serialKiller)
{
	// circle formula: (x - cp.x)^2 + (y - cp.y)^2 = r^2;
	// generator random amount of points on the circle: (at least 3 point)
	geometry_msgs::Point temp;
	srand(time(NULL));
	temp.x = 0;
	temp.z = centerPoint.z;
	for(int i = 0; i < num; i++)
	{
		temp.y = sqrt(radius * radius - (temp.x - centerPoint.x) * (temp.x - centerPoint.x)) + centerPoint.y;
		serialKiller.push_back(temp);
		temp.y = -sqrt(radius * radius - (temp.x - centerPoint.x) * (temp.x - centerPoint.x)) + centerPoint.y;
		serialKiller.push_back(temp);
		if(temp.x < radius + centerPoint.x)
			temp.x += stride;
		else
			break;
	}
}

    










