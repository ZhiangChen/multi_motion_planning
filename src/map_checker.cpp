#include <ros/ros.h> 
#include <std_msgs/Float64.h> 
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PointStamped.h>
#include <iostream>
using namespace std;

nav_msgs::OccupancyGrid g_map;
bool got_map = false;
void myCallback(const nav_msgs::OccupancyGrid& map) 
{ 

  ROS_INFO("received the map."); 
  g_map = map;
  ROS_INFO("Width x Height: (%d x %d)", g_map.info.width, g_map.info.height);
  ROS_INFO("Origin: (%f, %f)", g_map.info.origin.position.x, g_map.info.origin.position.y);
  ROS_INFO("Resoluntion: %f",g_map.info.resolution);
  int n = g_map.data.size();
  ROS_INFO("Points: %d", n);
  geometry_msgs::PointStamped pt;
  got_map = true;

} 

int main(int argc, char **argv) 
{ 
  ros::init(argc,argv,"map_checker"); //name this node 
  // when this compiled code is run, ROS will recognize it as a node called "minimal_subscriber" 
  ros::NodeHandle n; 
  ros::Subscriber my_subscriber_object= n.subscribe("/map",1,myCallback); 
  ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("/my_map", 1);

  int x;
  int8_t y;
  while(ros::ok())
  {
    ros::spinOnce(); 
    if(got_map)
    {
      cout<<"Input the point's position: ";
      cin>>x;
      cout<<"Input the occupancy: ";
      cin>>y;
      g_map.data[x] = y;
      map_pub.publish(g_map); 
    }
  }
  

  return 0; 
} 
