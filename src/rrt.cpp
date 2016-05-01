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
	extendRRT();
	connectRRT();
	mergeRRT();
	return true;
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

}

bool RRT::extendRRT()
{

}

bool RRT::connectRRT()
{

}

bool RRT::mergeRRT()
{

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
		ROS_INFO("Out of C-Space!");
		return false;
	}
	if (robot.point.y<(0.0+safe_rds) || robot.point.y>(height_-safe_rds))
	{
		ROS_INFO("Out of C-Space!");
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
			ROS_INFO("Point Got Collided!");
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
			ROS_INFO("Line Got Collided!");
			//cout<<Mid.point.x<<", "<<Mid.point.y<<endl;
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

bool RRT::checkVertex(Vertex v)
{
	int n=v.n;
	// check if points are valid in map
	for (int j=0; j<n; j++)
	{
		if(!checkPoint(v.point[j]))
		{
			ROS_INFO("Invalid Vertex!");
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
				ROS_INFO("Invalid Vertex!");
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
			ROS_INFO("Invalid Edge");
			ROS_INFO("Invalid Line %d", j);
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
 			ROS_INFO("Invalid Edge");
			return false;
 		}
	}
	return true;

}


/*
void RRT::getQrand(std::vector<geometry_msgs::PointStamped> &robots)
{
	robots.resize(nm_);
	for(int j=0; j<nm_; j++)
	{
		robots[j]=randomPoint();
	}
	checkCollision(robots);
}*/