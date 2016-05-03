/// Rapidly-Exploring Random Tree (rrt) for Mulitple Mobile Robots
/// Zhiang Chen, 4/2016
/// CECS499 Course Project
/// Prof. M. Cenk Çavuşoğlu

/*The MIT License (MIT)

Copyright (c) 2016 Zhiang Chen

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.*/

#ifndef RRT_H_
#define RRT_H_

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <string>

#define SimpleRobot
#ifdef SimpleRobot
	#define robot_x 0.35
	#define robot_y 0.35
	#define robot_z 0.25
	#define safe_rds 0.25
#endif


	
// RRT parameters
#define collision_check_pts 50
#define step_size 0.2

struct Vertex
{
	Vertex(int d):n(d) // the number of the robots
	{
		point.resize(n);
		connectivity = 0;
		parent = -2;
	};
	int n;
	int parent;
	std::vector<geometry_msgs::PointStamped> point;
	int connectivity;
	void displayVertex();
};


struct Edge
{
	Edge(int d):n(d),start_vertex(d),end_vertex(d) // the number of the robots
	{};
	int n;
	Vertex start_vertex;
	Vertex end_vertex;
	void displayEdge();
};

struct Tree
{
	Tree()
	{
		id = "None";
		partner_id = "None";
		connection = false;
	}
	std::string id;
	std::string partner_id;
	bool connection;
	std::vector<Vertex> vertexs;
	std::vector<Edge> edges;
	void getPath(std::vector<nav_msgs::Path> &path);
	void displayTree();
	void displayInfo();
};

class RRT
{
public:
	RRT(int nm, int n_q_rand);
	void setGoal(geometry_msgs::PointStamped goal, int index);
	void swapPositions();
	bool runRRT();
	bool getPath();
	bool getPath(std::vector<nav_msgs::Path> &Path);
	bool getPaht2(nav_msgs::Path &path);
	void displayPath();
//private:
	void getMap();
	void mapCallback(const nav_msgs::OccupancyGrid& map);
	void getCSpace();
	void getInit();
	bool checkCollision(std::vector<geometry_msgs::PointStamped> robots);
	double getDistance(geometry_msgs::PointStamped robot1, geometry_msgs::PointStamped robot2);
	bool checkPoint(geometry_msgs::PointStamped point);
	bool checkLine(geometry_msgs::PointStamped point1, geometry_msgs::PointStamped point2);
	geometry_msgs::PointStamped randomPoint();
	bool checkVertex(Vertex v);
	bool checkEdge(Edge e);
	void getRandomVertex(Vertex &v);
	int findClosestVertex(Tree t, Vertex v); // return the index of closest vertex
	std::vector<nav_msgs::Path> path_;

	bool buildRRT();
	bool extendRRT(Tree &t, Vertex &v);
	bool mergeRRT(Tree &t, Vertex v);
	
	
	int nm_; 
	std::vector<bool> got_goal_table_;
	bool got_goal_;
	bool got_path_;
	ros::NodeHandle nh_;
	bool got_map_;
	bool connected_;
	ros::Subscriber map_sub_;
	nav_msgs::OccupancyGrid map_;
	double rsl_;
	double width_, height_;
	int width_px_, height_px_;
	std::vector<double> cspace_;
	int n_q_rand_;
	std::vector<ros::Publisher> path_pub_;
	

	std::vector<geometry_msgs::PointStamped> init_;
	std::vector<geometry_msgs::PointStamped> goal_;
	
	Tree T_init_; 
	Tree T_goal_; 

};


#endif
