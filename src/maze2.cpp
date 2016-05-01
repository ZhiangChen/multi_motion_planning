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
	char a;
	geometry_msgs::PointStamped robot0,robot1;
while(ros::ok())
{	
	/* // test checkPoint
	cout<<"robot0 x,y";
	cin>>robot0.point.x>>robot0.point.y;
	Rrt.checkPoint(robot0);
	*/

	/*// test randPoint & checkLine
	robot0= Rrt.randomPoint();
	cout<<"robot0: "<<robot0.point.x<<", "<<robot0.point.y<<endl;
	robot1= Rrt.randomPoint();
	cout<<"robot1: "<<robot1.point.x<<", "<<robot1.point.y<<endl;
	Rrt.checkLine(robot0,robot1);
	cout<<"checking points:";
	Rrt.checkPoint(robot0);
	Rrt.checkPoint(robot1);
	cout<<"......";
	cin>>a;*/

	/* // test checkVertex
	cout<<"robot0 x,y: ";
	cin>>robot0.point.x>>robot0.point.y;
	cout<<"robot1 x,y: ";
	cin>>robot1.point.x>>robot1.point.y;
	Vertex v(2);
	v.point[0]=robot0;
	v.point[1]=robot1;
	Rrt.checkVertex(v);*/

	// test checkEdge
	cout<<"start: "<<endl;
	cout<<"robot0 x,y: ";
	cin>>robot0.point.x>>robot0.point.y;
	cout<<"robot1 x,y: ";
	cin>>robot1.point.x>>robot1.point.y;
	Vertex s_v(2);
	s_v.point[0] = robot0;
	s_v.point[1] = robot1;
	cout<<"end: "<<endl;
	cout<<"robot0 x,y: ";
	cin>>robot0.point.x>>robot0.point.y;
	cout<<"robot1 x,y: ";
	cin>>robot1.point.x>>robot1.point.y;
	Vertex e_v(2);
	e_v.point[0] = robot0;
	e_v.point[1] = robot1;
	Edge e(2);
	e.start_vertex = s_v;
	e.end_vertex = e_v;
	Rrt.checkEdge(e);

}




	return 0; 
} 
