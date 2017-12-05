#include <geometry_msgs/Point.h>
#include <fstream>
#include <string>
#include <vector>
#include <stdio.h>
#include <stdlib.h>

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

