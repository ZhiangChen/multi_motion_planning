#include <ros/ros.h> 
#include <std_msgs/Float64.h> 
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PointStamped.h>
#include <iostream>
#include <multi_motion_planning/rrt.h>

using namespace std;



int main(int argc, char **argv) 
{ 
  ros::init(argc,argv,"maze2"); 
  ros::NodeHandle n; 
  RRT Rrt(2,5);


  return 0; 
} 
