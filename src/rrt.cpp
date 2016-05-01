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
		if(!got_goal_table_[index])
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
	int width_px, height_px;
	width_px = map_.info.width;
	height_px = map_.info.height;
	ROS_INFO("Width x Height: (%d x %d) pixels", width_px, height_px);
	double x,y;
	x = map_.info.origin.position.x;
	y = map_.info.origin.position.y;
	ROS_INFO("Origin: (%f, %f)", x, y);
	double rsl;
	rsl = map_.info.resolution;
	ROS_INFO("Resoluntion: %f",rsl);
	int n = map_.data.size();
	ROS_INFO("Points: %d", n);
	width_ = (width_px) * rsl;
	height_ = (height_px) * rsl;
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
			cspace_[j++]= width_ - 2*safe_rds;
			cspace_[j++]= height_ - 2*safe_rds;
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
