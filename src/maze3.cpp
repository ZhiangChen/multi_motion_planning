#include <ros/ros.h> 
#include <std_msgs/Float64.h> 
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PointStamped.h>
#include <iostream>
#include <multi_motion_planning/rrt.h>

using namespace std;



int main(int argc, char **argv) 
{ 
	ros::init(argc,argv,"maze3"); 
	ros::NodeHandle n; 
	RRT Rrt(3,5);
	Rrt.swapPositions();
	Rrt.buildRRT();

	Rrt.runRRT();
	Rrt.getPath();
	geometry_msgs::PointStamped robot0,robot1,robot2;
	while(ros::ok())
	{
		Rrt.displayPath();
		ros::Duration(0.1).sleep();
	}

	return 0; 


  return 0; 
} 
