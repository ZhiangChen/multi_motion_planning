#include <ros/ros.h> 
#include <std_msgs/Float64.h> 
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PointStamped.h>
#include <iostream>
using namespace std;

nav_msgs::OccupancyGrid g_map;
bool got_map = false;
int g_width_px;
int g_height_px;
double g_rsl;
double g_x;
double g_y;
double g_width;
double g_height;
void myCallback(const nav_msgs::OccupancyGrid& map) 
{ 
  ROS_INFO("Received the map."); 
  g_map = map;
  g_width_px = g_map.info.width;
  g_height_px = g_map.info.height;
  ROS_INFO("Width x Height: (%d x %d) pixels", g_width_px, g_height_px);
  g_x = g_map.info.origin.position.x;
  g_y = g_map.info.origin.position.y;
  ROS_INFO("Origin: (%f, %f)", g_x, g_y);
  g_rsl = g_map.info.resolution;
  ROS_INFO("Resoluntion: %f",g_rsl);
  int n = g_map.data.size();
  ROS_INFO("Points: %d", n);
  g_width = (g_width_px+1) * g_rsl;
  g_height = (g_height_px+1) * g_rsl;
  ROS_INFO("Width x Height: (%d x %d) meters", g_width, g_height;
  got_map = true;
} 

int main(int argc, char **argv) 
{ 
  ros::init(argc,argv,"collision_checker"); 
  ros::NodeHandle n; 
  ros::Subscriber my_subscriber_object= n.subscribe("/map",1,myCallback); 
  ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("/my_map", 1);

  double x,y;
  int i_x,i_y;
  while(ros::ok())
  {
    ros::spinOnce(); 
    if(got_map)
    {
      cout<<"Input the point's position: ";
      cin>>x>>y;
      i_x = (x-g_x)/g_rsl;
      i_y = (y-g_y)/g_rsl;
      cout<<"indice: "<<i_x<<", "<<i_y<<endl;
      cout<<"Occupancy: "<<int(g_map.data[g_width_px * (i_y)+ i_x])<<endl; 
    }
  }
  

  return 0; 
} 
