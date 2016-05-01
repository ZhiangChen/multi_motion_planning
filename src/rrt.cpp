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
	double theta = 2*M_PI/collision_check_pts;
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

bool RRT::checkVertex(Vertex v)
{
	geometry_msgs::PointStamped robot = v.point;
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
	double theta = 2*M_PI/collision_check_pts;
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
			ROS_INFO("Vertex Got Collided!");
			return false;
		}
	}
	return true;
}

bool checkEdge(Edge edge)
{
	
}