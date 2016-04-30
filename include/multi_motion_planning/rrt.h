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

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PolygonStamped.h>

#ifndef RRT_H_
#define RRT_H_

struct Vertex
{
	geometry_msgs::PointStamped point;
	int connectivity;
}

struct Edge
{
	geometry_msgs::PointStamped start_point;
	geometry_msgs::PointStamped end_point;
}

struct Tree
{
	std::vector<Vertex> vertexs;
	std::vector<Edge> edges;
	nav_msgs::Path getTree();
}

class RRT
{
public:
	RRT(int nm);
	void configInit(int n_q_rand, double step_size);
	void setGoal(geometry_msgs::PoseStamped goal, int index);
	bool runRRT();
	bool getPath(nav_msgs::Path &path);
	bool getPaht2(nav_msgs::Path &path);
	bool displayTrees();
private:
	void getMap();
	void getCSpace();
	bool checkCollision();
	
	void buildRRT();
	void extendRRT();
	void connectRRT();
	void mergeRRT();
	
	int nm_; 
	bool got_config_;
	bool got_goal_;
	bool got_path_;
	
	std::vector<Vertex> vertexs_;
	std::vector<Edge> edges_;

};

#endif
