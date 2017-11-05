#include "circular_pos_generator.h"
#include <visualization_msgs/Marker.h>

void read_from_file(std::string dir, std::vector<geometry_msgs::Point>& skeleton, double flag)
{
	std::ifstream in(dir.c_str());
	if(in == NULL)
	{
		printf("Cannot open file\n");
		exit(-1);
	}
	std::string str;
	std::string temp;
	std::vector<std::vector<double> > data;
	// read file
	while(std::getline(in, str))
	{
		std::stringstream ss(str);
		std::vector<double> skeleton_point(24 * 3 + 1, 0);
		int i = 0;
		// tokenizer
		while(std::getline(ss, temp, ','))
		{
					skeleton_point[i] = std::atof(temp.c_str());
					i++;
		}
		if(skeleton_point.back() == flag)
		{
			// get rid of the flag
			skeleton_point.pop_back();
			data.push_back(skeleton_point);
		}
	}
	
	
	// transfer data to point
	for(unsigned long i = 0; i < data.size(); i++)
	{
		geometry_msgs::Point point_temp;
		for(unsigned long j = 0; j < data[i].size(); j++)
		{
			if(j % 3 == 0) point_temp.y = data[i][j];
			else if(j % 3 == 1) point_temp.z = data[i][j] + 0.7;
			else if(j % 3 == 2)
			{
				point_temp.x = data[i][j];
				skeleton.push_back(point_temp);
			}
		}
	}
}



int main( int argc, char** argv )
{
	std::string skeleton_data1_dir = "test3.txt";
	// string skeleton_data2_dir = "/skeleton_data/test2.txt";
	std::vector<geometry_msgs::Point> skeleton0, skeleton1;
	read_from_file(skeleton_data1_dir, skeleton0, 0.0);
	read_from_file(skeleton_data1_dir, skeleton1, 1.0);
	
	// skeleton of neck pos 
	geometry_msgs::Point p1 = skeleton0[0]; 
	geometry_msgs::Point p2 = skeleton1[0];
	p2.z = p1.z;
	ROS_INFO("p1: (%f, %f, %f), p2: (%f, %f, %f)", p1.x, p1.y, p1.z, p2.x, p2.y, p2.z);
	// on screen plane the projection point:
	point2D pp1, pp2;
	pp1.x = 200, pp2.x = 400;
	pp1.y = 240, pp2.y = 240;
	
	int num = 10;
	std::vector<geometry_msgs::Point> serialKiller;
	
	generateSerialPoints(num, p1, p2, pp1, pp2, serialKiller);
	
	ros::init(argc, argv, "circular_pos");
	ros::NodeHandle nh;
	ros::Publisher circular_pos_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	ros::Rate rate(30.0);
	
	while(ros::ok())
	{
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

