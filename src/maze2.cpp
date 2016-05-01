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
  //Rrt.swapPositions();
  geometry_msgs::PointStamped robot0,robot1;
  robot0.point.x=4.0;
  robot0.point.y=1.21;
  robot1.point.x=2;
  robot1.point.y=7;
  std::vector<geometry_msgs::PointStamped> robots;
  robots.push_back(robot0);
  robots.push_back(robot1);
  Rrt.checkCollision(robots);


  return 0; 
} 
