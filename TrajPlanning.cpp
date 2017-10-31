#include<ros.h>
#include<stdio.h>


// screen size: 640 * 480
// suppose:
// focal length: fx
// camera center: c (0, 0, 0)
// screen position: s (0, 0, fx);

// exchanging coordinate needed(between world coord and camera coord)

// fixed postion of p1 and p2:
// p1:(p1.x, p1.y, p1.z)
// p2:(p2.x, p2.y, p2.z)

// on screen plane, we can get the projection point of p1 and p2: pp1 and pp2
// vec<cp1> = (p1.x - c.x, p1.y - c.y, p1.z - c.z)
// vec<cpp1> = (pp1.x - c.x, pp1.y - c.y, pp1.z - c.z), in which pp1.z = fx
// vec<cp1> = t * vec<cpp1> 

<geometry_msgs/Point> pp1, pp2;

double t1 = p1.z / fx;
pp1.x = p1.x / t1;
pp1.y = p1.y / t1;
pp1.z = fx;

double t2 = p2.z / fx;
pp2.x = p2.x / t1;
pp2.y = p2.y / t1;
pp1.z = fx;

// fixed pos: pp1 and pp2(in camera coordinate sys, the coordinate has been changed)
// calculate radius and center of the circle(pp1, pp2 and c)


// calculate o
// vec<pp1o> = m * vec<p1c1>, vec<pp2o> = n * vec<p2c2>
std::vector<geometry_msgs/Point> pd;
pd.push_back(pp1);
pd.push_back(pp2);
pd.push_back(c);
<geometry_msgs/Point> o;
solveCenterPointOfCircle(pd, o);
double radius = sqrt(o.x^2 + o.y^2 + o.z^2);




int solveCenterPointOfCircle(std::vector<geometry_msgs/Point> pd, <geometry_msgs/Point>& centerpoint)
{
	double a1, b1, c1, d1;
	double a2, b2, c2, d2;
	double a3, b3, c3, d3;

	double x1 = pd[0].x, y1 = pd[0].y, z1 = pd[0].z;
	double x2 = pd[1].x, y2 = pd[1].y, z2 = pd[1].z;
	double x3 = pd[2].x, y3 = pd[2].y, z3 = pd[2].z;

	a1 = (y1*z2 - y2*z1 - y1*z3 + y3*z1 + y2*z3 - y3*z2);
	b1 = -(x1*z2 - x2*z1 - x1*z3 + x3*z1 + x2*z3 - x3*z2);
	c1 = (x1*y2 - x2*y1 - x1*y3 + x3*y1 + x2*y3 - x3*y2);
	d1 = -(x1*y2*z3 - x1*y3*z2 - x2*y1*z3 + x2*y3*z1 + x3*y1*z2 - x3*y2*z1);

	a2 = 2 * (x2 - x1);
	b2 = 2 * (y2 - y1);
	c2 = 2 * (z2 - z1);
	d2 = x1 * x1 + y1 * y1 + z1 * z1 - x2 * x2 - y2 * y2 - z2 * z2;

	a3 = 2 * (x3 - x1);
	b3 = 2 * (y3 - y1);
	c3 = 2 * (z3 - z1);
	d3 = x1 * x1 + y1 * y1 + z1 * z1 - x3 * x3 - y3 * y3 - z3 * z3;

	centerpoint.x = -(b1*c2*d3 - b1*c3*d2 - b2*c1*d3 + b2*c3*d1 + b3*c1*d2 - b3*c2*d1)
		/(a1*b2*c3 - a1*b3*c2 - a2*b1*c3 + a2*b3*c1 + a3*b1*c2 - a3*b2*c1);
	centerpoint.y =  (a1*c2*d3 - a1*c3*d2 - a2*c1*d3 + a2*c3*d1 + a3*c1*d2 - a3*c2*d1)
		/(a1*b2*c3 - a1*b3*c2 - a2*b1*c3 + a2*b3*c1 + a3*b1*c2 - a3*b2*c1);
	centerpoint.z = -(a1*b2*d3 - a1*b3*d2 - a2*b1*d3 + a2*b3*d1 + a3*b1*d2 - a3*b2*d1)
		/(a1*b2*c3 - a1*b3*c2 - a2*b1*c3 + a2*b3*c1 + a3*b1*c2 - a3*b2*c1);

	return 0;
}













