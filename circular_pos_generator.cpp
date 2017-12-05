#include "circular_pos_generator.h"
#include "read_from_file.h"
#include "waypoint_send.h"
#include <visualization_msgs/Marker.h>
#include <std_msgs/Int8.h>

int count = 0;
int threshold = 200;
double movement = 0.005;
std::vector<geometry_msgs::Point> waypoints;
geometry_msgs::Point center;
geometry_msgs::Point p1; 
geometry_msgs::Point p2;
int trigger = 1;
ros::Publisher waypoint_pub;
nav_msgs::Path path;

void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    path.poses.clear();
		//nav_msgs::Path path;
		geometry_msgs::Point dronePos;
		dronePos.x = msg->pose.pose.position.x, dronePos.y = msg->pose.pose.position.y, dronePos.z = msg->pose.pose.position.z;
		//ROS_INFO("current pos: (%f, %f, %f)", dronePos.x, dronePos.y, dronePos.z);
		// set gimbal control;
		setYaw(center, dronePos);
    setPath(waypoints, trigger, path, dronePos);
    
    if(trigger == 1 && path.poses.size() > 0)
    {
    	waypoint_pub.publish(path);
 	    // ros::Duration(2.0).sleep(); 
		}
	  
}



void TriggerCallback(const std_msgs::Int8::ConstPtr &msg)
{
      waypoint_pub.publish(path);
 	    //ros::Duration(1.0).sleep(); 

}

int main( int argc, char** argv )
{
	std::string skeleton_data1_dir = "test3.txt";
	// string skeleton_data2_dir = "/skeleton_data/test2.txt";
	std::vector<geometry_msgs::Point> skeleton0, skeleton1;
	read_from_file(skeleton_data1_dir, skeleton0, 0.0);
	read_from_file(skeleton_data1_dir, skeleton1, 1.0);
	
	p1 = skeleton0[0]; 
	p2 = skeleton1[0];
	
	p1.x = 1; p1.y = 2;
	p2.x = -3; p2.y = 5;
	p2.z = p1.z = 2;
	center.x = (p1.x + p2.x) / 2;
	center.y = (p1.y + p2.y) / 2;
	center.z = p1.z;
	
	// on screen plane the projection point:
	point2D pp1, pp2;
	pp1.x = 200, pp2.x = 400;
	pp1.y = 240, pp2.y = 240;
	
	ros::init(argc, argv, "circular_pos");
	ros::NodeHandle nh;
	ros::Publisher circular_pos_pub = nh.advertise<visualization_msgs::Marker>("circular_pos", 10);
	ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("marker", 10);
	waypoint_pub = nh.advertise<nav_msgs::Path>("/waypoint_generator/waypoints",10);
  ros::Subscriber uav_sub = nh.subscribe("/visual_slam/odom", 10, OdomCallback);
  ros::Subscriber trigger_sub = nh.subscribe("/toric_trigger", 10, TriggerCallback);
	ros::Rate rate(30.0);
	
	while(ros::ok()) 
	{
			if(count < threshold)
			count++;    
	    
	    
		// skeleton of neck pos 
		if(count < threshold){
		 	p2.x += 2 * movement;
		}
		
		center.x = (p1.x + p2.x) / 2;
		center.y = (p1.y + p2.y) / 2;
		
		// ROS_INFO("head: (%f, %f, %f)", p2.x, p2.y, p2.z);		
		// calculate traj
		double radius;
		geometry_msgs::Point centerPoint;
		calCircleParameters(p1, p2, pp1, pp2, radius, centerPoint);
		// sample the circle for display
		int num = 20; // sample number
		generateSerialPoints(num, radius, centerPoint, p1, p2, waypoints);		
		
		
		// *********************************************** display the traj ****************************************** 
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
    for (unsigned long i = 0; i < waypoints.size(); ++i){
    		traj.points.push_back(waypoints[i]);
    }
    circular_pos_pub.publish(traj);
    
    
    //*********************************************** display the skeleton **************************************
    visualization_msgs::Marker skeleton_joint0, skeleton_joint1, head1, head2;
    skeleton_joint0.header.frame_id = skeleton_joint1.header.frame_id = head1.header.frame_id = head2.header.frame_id =  "/map";
    skeleton_joint0.header.stamp = skeleton_joint1.header.stamp = head1.header.stamp = head2.header.stamp = ros::Time::now();
		// namespace is used to create used name for this marker.
    skeleton_joint0.ns = skeleton_joint1.ns = head1.ns = "skeleton";
    skeleton_joint0.action = skeleton_joint1.action = head1.action = head2.action = visualization_msgs::Marker::ADD;
    skeleton_joint0.pose.orientation.w = skeleton_joint1.pose.orientation.w = head1.pose.orientation.w = head2.pose.orientation.w= 1.0;

    skeleton_joint0.id = 0;
		skeleton_joint1.id = 1;
		head1.id = 4;
		head2.id = 4;
/*
    skeleton_joint0.type = skeleton_joint1.type = visualization_msgs::Marker::SPHERE_LIST;
    // POINTS markers use x and y scale for width/height respectively
    skeleton_joint0.scale.x = skeleton_joint1.scale.x = 0.1;
    skeleton_joint0.scale.y = skeleton_joint1.scale.y = 0.1;
    skeleton_joint0.scale.z = skeleton_joint1.scale.z = 0.1;
    // skeleton_joint0 is green
    skeleton_joint0.color.g = 1.0f;
    skeleton_joint0.color.a = 1.0f;
    // skeleton_joint1 is blue
    skeleton_joint1.color.g = 1.0f;
    skeleton_joint1.color.a = 1.0f;
*/    
        
    head1.type = visualization_msgs::Marker::SPHERE_LIST;
    head2.type = visualization_msgs::Marker::SPHERE_LIST;


    
        
    head1.scale.x = 1.0;
    head1.scale.y = 1.0;
    head1.scale.z = 1.0;

		head2.scale.x = 1.0;
    head2.scale.y = 1.0;
    head2.scale.z = 1.0;


		// head1 is red
		head1.color.r = 1.0f;
		head1.color.a = 1.0f;
		// head1 is blue
		head2.color.b = 1.0f;
		head2.color.a = 1.0f;
		
		// head is p1, p2
		head1.points.push_back(p1);
		head2.points.push_back(p2);

/*		
    // Create the vertices for the points and lines
    for (unsigned long i = 0; i < skeleton0.size(); ++i)
    {
    	if(skeleton0[i].x != 0.0f && skeleton0[i].y != 0.0f)
    		{
    			skeleton_joint0.points.push_back(skeleton0[i]);
				}
    }
		
		for (unsigned long i = 0; i < skeleton1.size(); ++i)
		{
			if(skeleton1[i].x != 0.0f && skeleton1[i].y != 0.0f)
			{
					if(count < threshold)
				 		skeleton1[i].x += movement;
					skeleton_joint1.points.push_back(skeleton1[i]);
			}	
		}
		



    
    marker_pub.publish(skeleton_joint0);
		marker_pub.publish(skeleton_joint1);
		*/
		marker_pub.publish(head1);
		marker_pub.publish(head2);
		
		ros::spinOnce();
		
    rate.sleep();

	}
	
}

