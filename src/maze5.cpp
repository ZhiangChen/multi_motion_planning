#include <ros/ros.h> 
#include <std_msgs/Float64.h> 
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PointStamped.h>
#include <iostream>
#include <multi_motion_planning/rrt.h>

using namespace std;



int main(int argc, char **argv) 
{ 
	ros::init(argc,argv,"maze5"); 
	ros::NodeHandle n; 
	RRT Rrt(5,3);
	Rrt.swapPositions();
	ros::Time t_s = ros::Time::now();
	Rrt.runRRT();
	Rrt.getPath();
	ros::Time t_e = ros::Time::now();
	cout<<t_e-t_s<<endl;
	while(ros::ok())
	{
		Rrt.displayPath();
		ros::Duration(1).sleep();
	}

	return 0; 


  return 0; 
} 
