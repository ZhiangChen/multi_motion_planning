#include <multi_motion_planning/rrt.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <string>

using namespace std;

RRT::RRT(int nm, int n_q_rand):nm_(nm),n_q_rand_(n_q_rand)
{
    got_map_ = false;
    got_goal_table_.resize(nm_);
    got_goal_ = false;
    for(int j=0; j<nm_; j++)
    {
    	got_goal_table_[j] = false;
    }
    init_.resize(nm_);
    goal_.resize(nm_);
    getMap();
    getCSpace();
    getInit();
    srand(time(NULL));
    connected_ = false;
    T_init_.id = "Init_Tree";
    T_goal_.id = "Goal_Tree";
    path_pub_.resize(nm_);
    std::vector<string> topic;
	topic.resize(nm_);
	for (int j=0; j<nm_; j++)
	{
		topic[j]="path"+boost::to_string(j);
		path_pub_[j]=nh_.advertise<nav_msgs::Path>(topic[j],1,true);
	}
	got_path_ = false;
}

void RRT::setGoal(geometry_msgs::PointStamped goal, int index)
{
	goal_[index] = goal;
	got_goal_table_[index] = true;
	double x,y;
	x = goal_[index].point.x;
	y = goal_[index].point.y;
	ROS_INFO("The robot%d's goal position is (%f,%f).",index,x,y);
	int j;
	for(j=0; j<nm_; j++)
	{
		if(!got_goal_table_[j])
			break;
	}
	if(j==nm_)
		got_goal_ = true;
}

void RRT::swapPositions()
{
	int next;
	double x,y;
	for(int j=0; j<nm_; j++)
	{
		next = j+1;
		if(next==nm_)
			next=0;
		goal_[j] = init_[next];
		x = goal_[j].point.x;
		y = goal_[j].point.y;
		ROS_INFO("The robot%d's goal position is (%f,%f).",j,x,y);
	}
	got_goal_ = true;
}

bool RRT::runRRT()
{
	if(!got_goal_)
	{
		ROS_ERROR("Invalid Goals!");
		return false;
	}
	buildRRT();
	int i=0;
	Vertex Qrand(nm_);
	do
	{
		getRandomVertex(Qrand, T_goal_.vertexs[0]);
		if(extendRRT(T_init_,Qrand))
			if(mergeRRT(T_goal_,Qrand))
				break;
		getRandomVertex(Qrand, T_init_.vertexs[0]);
		if(extendRRT(T_goal_,Qrand))
			if(mergeRRT(T_init_,Qrand))
				break;
		if(i++ > 500000)
		{
			ROS_ERROR("Time out!");
			return false;
		}		
	}while(!connected_);
	ROS_INFO("Connected! %d iterations", i);
	
	T_init_.connection = true;
	T_init_.partner_id = "Goal_Tree";
	T_goal_.connection = true;
	T_goal_.partner_id = "Init_Tree";
	T_init_.displayInfo();
	T_goal_.displayInfo();
	std::vector<nav_msgs::Path> path;
	T_init_.getPath(path);
	return true;
}

bool RRT::getPath()
{
	if (!connected_)
	{
		ROS_ERROR("Cannot get path without connected tree!");
		return false;
	}
	//ROS_INFO("Getting path");
	std::vector<nav_msgs::Path> P_init, P_goal, Path;
	Path.resize(nm_);
	T_init_.getPath(P_init);
	T_goal_.getPath(P_goal);
	int pts_i = P_init[0].poses.size();
	int pts_g = P_goal[0].poses.size();
	int pts = pts_i + pts_g;
	for (int j=0; j<nm_; j++)
	{
		Path[j].header.frame_id = "map";
		Path[j].poses.resize(pts);
	}
	// integrate init path
	for (int j=0; j<pts_i; j++)
	{
		for (int i=0; i<nm_; i++)
		{
			Path[i].poses[j].pose.position
			= P_init[i].poses[pts_i-1-j].pose.position;
			Path[i].poses[j].header.frame_id = "map";
		}
	}
	// integrate goal path
	for (int j=pts_i; j<pts; j++)
	{
		for (int i=0; i<nm_; i++)
		{
			Path[i].poses[j].pose.position
			= P_goal[i].poses[j-pts_i].pose.position;
			Path[i].poses[j].header.frame_id = "map";
		}
	}
	path_ = Path;
	got_path_ = true;	
}

bool RRT::getPath(std::vector<nav_msgs::Path> &Path)
{
	if (!connected_)
	{
		ROS_ERROR("Cannot get path without connected tree!");
		return false;
	}
	std::vector<nav_msgs::Path> P_init, P_goal;
	Path.resize(nm_);
	T_init_.getPath(P_init);
	T_goal_.getPath(P_goal);
	int pts_i = P_init[0].poses.size();
	int pts_g = P_goal[0].poses.size();
	int pts = pts_i + pts_g;
	for (int j=0; j<nm_; j++)
	{
		Path[j].header.frame_id = "map";
		Path[j].poses.resize(pts);
	}
	// integrate init path
	for (int j=0; j<pts_i; j++)
	{
		for (int i=0; i<nm_; i++)
		{
			Path[i].poses[j].pose.position
			= P_init[i].poses[pts_i-1-j].pose.position;
			Path[i].poses[j].header.frame_id = "map";
		}
	}
	// integrate goal path
	for (int j=pts_i; j<pts; j++)
	{
		for (int i=0; i<nm_; i++)
		{
			Path[i].poses[j].pose.position
			= P_goal[i].poses[j-pts_i].pose.position;
			Path[i].poses[j].header.frame_id = "map";
		}
	}
	path_ = Path;
	got_path_ = true;
	return true;
}

void RRT::displayPath()
{
	if(!got_path_)
	{
		ROS_ERROR("Cannot display path without getting the path");
		return;
	}
	for (int j=0; j<nm_; j++)
	{
		path_pub_[j].publish(path_[j]);
	}
}
/*****************************************************
***************     PRIVATE FUNCTIONS     ************               
******************************************************/

/********************************
******    CONFIGURTION    *******
********************************/
void RRT::getMap()
{
	map_sub_ = nh_.subscribe("/map",1,&RRT::mapCallback,this);
	ROS_INFO("Waiting for map loading...");
	while(ros::ok() && !got_map_)
	{
		ros::spinOnce();
	}
}

void RRT::mapCallback(const nav_msgs::OccupancyGrid& map)
{
	ROS_INFO("Received the map."); 
	map_ = map;
	width_px_ = map_.info.width;
	height_px_ = map_.info.height;
	ROS_INFO("Width x Height: (%d x %d) pixels", width_px_, height_px_);
	double x,y;
	x = map_.info.origin.position.x;
	y = map_.info.origin.position.y;
	ROS_INFO("Origin: (%f, %f)", x, y);
	rsl_ = map_.info.resolution;
	ROS_INFO("Resoluntion: %f",rsl_);
	int n = map_.data.size();
	ROS_INFO("Points: %d", n);
	width_ = (width_px_) * rsl_;
	height_ = (height_px_) * rsl_;
	ROS_INFO("Width x Height: (%f x %f) meters", width_, height_);
	got_map_ = true;
}

void RRT::getCSpace()
{
	int d = nm_*2;
	#ifdef SimpleRobot
		cspace_.resize(d);
		for (int j=0; j<d;)
		{
			cspace_[j++]= width_ - 2.0*safe_rds;
			cspace_[j++]= height_ - 2.0*safe_rds;
		}
	#endif
	ROS_INFO("Got C-Space with %d dimensionalities.", d);
}

void RRT::getInit()
{
	bool tferr;
	string map_str;
	string odom_str;
	string index;
	map_str = "/map";
	tf::TransformListener tf_listener;
	std::vector<tf::StampedTransform> tfs;
	tfs.resize(nm_);
	ros::Duration(2).sleep();
	for (int j=0; j<nm_; j++)
	{
		index = boost::to_string(j);
		odom_str = "/robot";
		odom_str += index;
		odom_str += "/odom";
		odom_str += index;
		ROS_INFO("waiting for tf robot%d and map...", j);
		tferr = true;
	    while (tferr) 
	    {
	        tferr = false;
	        try 
	        {
	            tf_listener.lookupTransform(map_str, odom_str, ros::Time(0), tfs[j]);
	        } 
	        catch (tf::TransformException &exception) 
	        {
	            ROS_ERROR("%s", exception.what());
	            tferr = true;
	            ros::Duration(0.5).sleep(); 
	            ros::spinOnce();
	        }
	    }
	    ROS_INFO("tf%d is good", j); 
	    init_[j].header.frame_id = "map";
	    init_[j].point.x = tfs[j].getOrigin()[0];
	    init_[j].point.y = tfs[j].getOrigin()[1];
	    double x,y;
	    x = init_[j].point.x;
	    y = init_[j].point.y;
	    ROS_INFO("The robot%d's initial position is (%f,%f).",j,x,y);
	}
}

/********************************
******     RRT  KERNEL    *******
********************************/
bool RRT::buildRRT()
{
	Vertex V_init(nm_);
	Vertex V_goal(nm_);
	V_init.point = init_;
	V_goal.point = goal_;
	V_init.connectivity = 1;
	V_goal.connectivity = 1;
	V_init.parent = -1;
	V_goal.parent = -1;
	T_init_.vertexs.push_back(V_init);
	T_goal_.vertexs.push_back(V_goal);
	Vertex Qrand(nm_);
	for(int j=0; j<n_q_rand_; j++)
	{
		do
		{
			getRandomVertex(Qrand, T_goal_.vertexs[0]);
		}while(!extendRRT(T_init_,Qrand));

		do
		{
			getRandomVertex(Qrand, T_init_.vertexs[0]);
		}while(!extendRRT(T_goal_,Qrand));
	}
	//T_init_.displayInfo();
	//T_goal_.displayInfo();
	//T_init_.displayTree();
	//T_goal_.displayTree();
}

bool RRT::extendRRT(Tree &t, Vertex &Qrand)
{
	int d= Qrand.n;
	Vertex Qnear(d);
	Vertex Qnew(d);
	int index = findClosestVertex(t,Qrand);
	Qnear = t.vertexs[index];
	// 1. try to connect Qnear and Qrand
	Edge e(d);
	e.start_vertex = Qnear;
	e.end_vertex = Qrand;
	if(checkEdge(e))
	{
		Qrand.connectivity++;
		Qrand.parent = index;
		t.vertexs.push_back(Qrand);
		t.edges.push_back(e);
		t.vertexs[index].connectivity++;
		return true;
	}
	// or 2. try to connect Qnear and Qnew
	std::vector<double> d_x;
	std::vector<double> d_y;
	d_x.resize(d);
	d_y.resize(d);
	//Qnew = Qnear;
	for (int j=0; j<d; j++)
	{
		d_x[j] = Qrand.point[j].point.x - Qnear.point[j].point.x;
		d_y[j] = Qrand.point[j].point.y - Qnear.point[j].point.y;

		Qnew.point[j].point.x = Qnear.point[j].point.x + d_x[j]*step_size;
		Qnew.point[j].point.y = Qnear.point[j].point.y + d_y[j]*step_size;
	}
	if(!checkVertex(Qnew))
	{
		return false;
	}
	e.start_vertex = Qnear;
	e.end_vertex = Qnew;
	if(!checkEdge(e))
	{
		return false;
	}
	Qnew.connectivity++;
	Qnew.parent = index;
	t.vertexs.push_back(Qnew);
	t.edges.push_back(e);
	t.vertexs[index].connectivity++;
	Qrand = Qnew;
	return true;
}


bool RRT::mergeRRT(Tree &t, Vertex v)
{
	int d=v.n;
	Vertex Qnear(d);
	Vertex Qrand(d);
	Qrand = v;
	int index = findClosestVertex(t,v);
	Qnear = t.vertexs[index];
	Edge e(d);
	e.start_vertex = Qnear;
	e.end_vertex = Qrand;
	if(checkEdge(e))
	{
		connected_ = true;
		Qrand.connectivity++;
		Qrand.parent = index;
		t.vertexs.push_back(Qrand);
		t.edges.push_back(e);
		t.vertexs[index].connectivity++;
		return true;
	}
	return false;
}
/********************************
******    RRT  ASSISTANT  *******
********************************/
bool RRT::checkCollision(std::vector<geometry_msgs::PointStamped> robots)
{
	int rest;
	double dist;
	// check the collision among robots
	for (int j=0; j<nm_; j++)
	{
		rest = nm_-j;
		for (int i=1; i<rest; i++)
		{
			dist = getDistance(robots[j],robots[j+i]);
			if (dist < safe_rds*2.0)
			{
				ROS_INFO("Got Collided!");
				return false;
			}
		}
	}
	// check the collision between robots and objects
	double theta = 2.0*M_PI/collision_check_pts;
	double angle;
	double  x,y;
	int i_x,i_y; // indices of x,y
	int occupancy;
	for (int j=0; j<nm_; j++)
	{
		// 1. robots should be in c-space
		if (robots[j].point.x<(0.0+safe_rds) || robots[j].point.x>(width_-safe_rds))
		{
			ROS_INFO("Out of C-Space!");
			return false;
		}
		if (robots[j].point.y<(0.0+safe_rds) || robots[j].point.y>(height_-safe_rds))
		{
			ROS_INFO("Out of C-Space!");
			return false;
		} 

		// 2. check the robot with the obstacles in map
		for (int i=0; i<collision_check_pts; i++)
		{
			angle = theta*i;
			x = robots[j].point.x + safe_rds*sin(angle); 
			y = robots[j].point.y + safe_rds*cos(angle);
			//cout<<x<<", "<<y<<endl; // display for debug
			// Assume the map's origin is (0,0)!!
			i_x = x/rsl_;
			i_y = y/rsl_;
			occupancy = int (map_.data[width_px_*(i_y)+i_x]);
			if (occupancy==100)
			{
				ROS_INFO("Got Collided with the %d-th point on robot%d!",i,j);
				return false;
			}
		}
	}
	return true;
}

double RRT::getDistance(geometry_msgs::PointStamped robot1, geometry_msgs::PointStamped robot2)
{
	double dist;
	double d_x,d_y;
	d_x = robot1.point.x - robot2.point.x;
	d_y = robot1.point.y - robot2.point.y;
	dist = sqrt(d_x*d_x + d_y*d_y);
	return dist;
}

bool RRT::checkPoint(geometry_msgs::PointStamped robot)
{
	// 1. robot should be in c-space
	if (robot.point.x<(0.0+safe_rds) || robot.point.x>(width_-safe_rds))
	{
		//ROS_INFO("Out of C-Space!");
		return false;
	}
	if (robot.point.y<(0.0+safe_rds) || robot.point.y>(height_-safe_rds))
	{
		//ROS_INFO("Out of C-Space!");
		return false;
	} 	
	// 2. check the collision between robot and the obstacles in map
	double theta = 2.0*M_PI/collision_check_pts;
	double angle;
	double  x,y;
	int i_x,i_y; // indices of x,y
	int occupancy;	
	for (int i=0; i<collision_check_pts; i++)
	{
		angle = theta*i;
		x = robot.point.x + safe_rds*sin(angle); 
		y = robot.point.y + safe_rds*cos(angle);
		//cout<<x<<", "<<y<<endl; // display for debug
		// Assume the map's origin is (0,0)!!
		i_x = x/rsl_;
		i_y = y/rsl_;
		occupancy = int (map_.data[width_px_*(i_y)+i_x]);
		if (occupancy==100)
		{
			//ROS_INFO("Point Got Collided!");
			return false;
		}
	}
	return true;
}

bool RRT::checkLine(geometry_msgs::PointStamped point1, geometry_msgs::PointStamped point2)
{
	int n;
	double dist,step;
	double d_x, d_y;
	geometry_msgs::PointStamped Mid;
	dist = getDistance(point1, point2);
	n = dist/rsl_ + 1;
	step = dist/n;
	d_x = (point2.point.x - point1.point.x)/n;
	d_y = (point2.point.y - point1.point.y)/n;
	for (int j=0; j<n; j++)
	{
		Mid.point.x = point1.point.x + (j+1)*d_x;
		Mid.point.y = point1.point.y + (j+1)*d_y;
		if(!checkPoint(Mid))
		{
			//ROS_INFO("Line Got Collided!");
			//cout<<Mid.point.x<<", "<<Mid.point.y<<endl;
			return false;
		}
	}
	return true;
}

bool RRT::checkVertex(Vertex v)
{
	int n=v.n;
	// check if points are valid in map
	for (int j=0; j<n; j++)
	{
		if(!checkPoint(v.point[j]))
		{
			//ROS_INFO("Invalid Vertex!");
			return false;
		}
	}

	// check self-collision
	int rest;
	double dist;
	// check the collision among robots
	for (int j=0; j<n; j++)
	{
		rest = n-j;
		for (int i=1; i<rest; i++)
		{
			dist = getDistance(v.point[j],v.point[j+i]);
			if (dist < safe_rds*2.0+0.01)
			{
				//ROS_INFO("Invalid Vertex!");
				return false;
			}
		}
	}
	return true;
}

bool RRT::checkEdge(Edge e)
{
	int n = e.n;
	// check lines are valid
	double longest_dist=-1;
	std::vector<double> dist;
	dist.resize(n);
	for(int j=0; j<n; j++)
	{
		if(!checkLine(e.start_vertex.point[j], e.end_vertex.point[j]))
		{
			//ROS_INFO("Invalid Edge");
			//ROS_INFO("Invalid Line %d", j);
			return false;
		}
		dist[j] = getDistance(e.start_vertex.point[j], e.end_vertex.point[j]);
		if(dist[j]>longest_dist)
		{
			longest_dist = dist[j];
		}
	}
	int n_s = longest_dist/rsl_ + 1; // the number of steps

	std::vector<double> d_x,d_y;
	d_x.resize(n);
	d_y.resize(n);
	for (int j=0; j<n; j++)
	{
		d_x[j] = (e.end_vertex.point[j].point.x - e.start_vertex.point[j].point.x)/n_s;
		d_y[j] = (e.end_vertex.point[j].point.y - e.start_vertex.point[j].point.y)/n_s;
	}

	// check self-collision
	Vertex Mid(n);
	for (int i=0; i<n_s; i++)
	{
		for (int j=0; j<n; j++)
		{
			Mid.point[j].point.x = e.start_vertex.point[j].point.x + (i+1)*d_x[j];
			Mid.point[j].point.y = e.start_vertex.point[j].point.y + (i+1)*d_y[j];
 		}		
 		if(!checkVertex(Mid))
 		{
 			//ROS_INFO("Invalid Edge");
			return false;
 		}
	}
	return true;
}

geometry_msgs::PointStamped RRT::randomPoint()
{
	geometry_msgs::PointStamped pt;
	do
	{
		pt.point.x = rand()*width_/RAND_MAX;
		pt.point.y = rand()*height_/RAND_MAX;
	}while(!checkPoint(pt));
	return pt;
}

void RRT::getRandomVertex(Vertex &v, Vertex goal)
{
	static int i = 0;
	if(++i == 20)
	{
		v = goal;
		i = 0;
		//cout<<"goal as random"<<endl;
		return;
	}
	int n = v.n;
	do
	{
		for (int j=0; j<n; j++)
		{
			v.point[j] = randomPoint();
		}		
	}while(!checkVertex(v));

}

int RRT::findClosestVertex(Tree t, Vertex v)
{
	int d = v.n;
	int n = t.vertexs.size();
	double shortest_dist=1000;
	double dist;
	std::vector<double> d_x;
	std::vector<double> d_y;
	d_x.resize(d);
	d_y.resize(d);
	int index;
	for (int j=0; j<n; j++)
	{
		dist = 0;
		for (int i=0; i<d; i++)
		{
			d_x[i] = t.vertexs[j].point[i].point.x - v.point[i].point.x;
			d_y[i] = t.vertexs[j].point[i].point.y - v.point[i].point.y;
			dist = dist + d_x[i]*d_x[i] + d_y[i]*d_y[i];
		}
		dist=sqrt(dist);
		if (dist<shortest_dist)
		{
			shortest_dist = dist;
			index = j;
		}
	}
	return index;
}

/*****************************************************
***************    STRUCTURE FUNCTIONS    ************               
******************************************************/
void Vertex::displayVertex()
{
	for (int i=0; i<n; i++)
	{
		cout<<"Robot"<<i<<" : "<<"("<<point[i].point.x<<", "<<point[i].point.y<<")"<<endl;
	}
	cout<<"Connectivity: "<<connectivity <<endl;
}

void Edge::displayEdge()
{
	cout<<"start: "<<endl;
	start_vertex.displayVertex();
	cout<<"end: "<<endl;
	end_vertex.displayVertex();
}

void Tree::displayInfo()
{
	cout<<"Name: "<<id<<endl;
	cout<<"Connection: "<<connection<<endl;
	cout<<"Connected tree: "<<partner_id<<endl;
	cout<<"The number of vertexs: "<<vertexs.size()<<endl;
	cout<<"The number of edges: "<<edges.size()<<endl;
}


void Tree::getPath(std::vector<nav_msgs::Path> &path)
{
	int n = vertexs.size();
	if (n<2)
	{
		ROS_ERROR("Cannot get the path from 1 vertex!");
		return;
	}
/*	cout<<"Init: "<<endl;
	vertexs[0].displayVertex();
	cout<<"Goal: "<<endl;
	vertexs[n-1].displayVertex();*/
	int d = vertexs[0].point.size();
	geometry_msgs::PoseStamped pose;
	path.resize(d);
	Vertex current_v = vertexs[n-1];
	//ROS_INFO("Getting Path...");
	cout<<id<<endl;
	for (int j=0; j<d; j++)
	{
		current_v = vertexs[n-1];
		while(1)
		{
			pose.pose.position = current_v.point[j].point;
			path[j].poses.push_back(pose);
			//current_v.displayVertex();
			current_v = vertexs[current_v.parent];
			if(current_v.parent == -1)
			{
				pose.pose.position = current_v.point[j].point;
				path[j].poses.push_back(pose);
			//	current_v.displayVertex();
				break;
			}
		}		
	}

	ROS_INFO("Got Path");
}

void Tree::displayTree()
{
	cout<<".........................."<<endl;
	cout<<"Tree name: "<<id<<endl;
	int n = vertexs.size();
	cout<<"Vertex: "<<endl;
	for (int j=0; j<n; j++)
	{
		vertexs[j].displayVertex();
	}
	n = edges.size();
	cout<<"Edge: "<<endl;
	for (int j=0; j<n; j++)
	{
		edges[j].displayEdge();
	}
}