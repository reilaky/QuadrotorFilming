#include "long_shot.h"
#include "read_from_file.h"
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{
	std::string skeleton_data1_dir = "test3.txt";
	// string skeleton_data2_dir = "/skeleton_data/test2.txt";
	std::vector<geometry_msgs::Point> skeleton0;
	read_from_file(skeleton_data1_dir, skeleton0, 0.0);
	
	// skeleton of neck pos as p1:
	geometry_msgs::Point p1 = skeleton0[0]; 
	// p2 is foot pos, here use p1's intersection with xoy plane:
	geometry_msgs::Point p2 = skeleton0[0];
	p2.z = 0;
	ROS_INFO("p1: (%f, %f, %f), p2: (%f, %f, %f)", p1.x, p1.y, p1.z, p2.x, p2.y, p2.z);
	

	// on screen plane (640 * 480)
	// the projection point:
	point2D pp1;
	pp1.x = 320, pp1.y = 400;
	pp2.x = pp1.x, pp2.y = 0;
	// calculate the circle, represented by(radius, centerPoint)
	double radius;
	std::vector<geometry_msgs::Point> centerPoint;
	calCircleParameters(p1, p2, pp1, pp2, radius, centerPoint);

	// sample the circle for display
	int num = 20; // sample number
	std::vector<geometry_msgs::Point> serialKiller;
	generateSerialPoints(num, p1, p2, pp1, pp2, serialKiller);

	// ros initialization
	ros::init(argc, argv, "circular_pos");
	ros::NodeHandle nh;
	ros::Publisher circular_pos_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	ros::Rate rate(30.0);
	
	while(ros::ok())
	{
		// for traj display:
		visualization_msgs::Marker traj;
	    traj.header.frame_id = "/map";
	    traj.header.stamp = ros::Time::now();
		// namespace is used to create used name for this marker.
	    traj.ns = "circular_pos";
	    traj.action = visualization_msgs::Marker::ADD;
	    traj.pose.orientation.w = 1.0;
	    traj.id = 0;
	    traj.type = visualization_msgs::Marker::SPHERE_LIST;

	    // POINTS markers use x and y scale for width/height respectively
	    traj.scale.x =  0.1;
	    traj.scale.y =  0.1;
	    traj.scale.z =  0.1;
	    // skeleton_joint0 is green
	    traj.color.r = 1.0f;
	    traj.color.a = 1.0f;

	    // Create the vertices for the points and lines
	    for (unsigned long i = 0; i < serialKiller.size(); ++i)
	    {
	    	traj.points.push_back(serialKiller[i]);
	    }
	      
	    circular_pos_pub.publish(traj);
	    rate.sleep();
		}
	
}

