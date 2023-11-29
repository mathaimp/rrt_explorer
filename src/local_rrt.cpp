#include "ros/ros.h"
#include "mtrand.h"
#include "helper.h"
#include <string>
#include <vector>

#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "visualization_msgs/Marker.h"
#include <tf/transform_listener.h>

// global vars
float startx, starty;
nav_msgs::OccupancyGrid mapData;
visualization_msgs::Marker lines, points;
geometry_msgs::PointStamped goal;
geometry_msgs::PointStamped click; //Selected Point


// Callback funcs
void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
	mapData =* msg;
}

void rvizCallBack(const geometry_msgs::PointStamped::ConstPtr& msg) {
	geometry_msgs::Point pnt;
	pnt.x = msg->point.x;
	pnt.y = msg->point.y;
	pnt.z = msg->point.z;
	points.points.push_back(pnt);
}


int main(int argc, char **argv) {
	// get a rand nos from mtrand
	MTRand_int32 irand();
	MTRand drand;

	ros::init(argc, argv, "local_rrt");
	ros::NodeHandle n;

	float init_x, init_y, range, eta;
	std::string map_topic, frame_topic;

	std::string ns;
	ns = ros::this_node::getName();
	ros::param::param<float>(ns+"/eta", eta, 0.5);
	ros::param::param<std::string>(ns+"/map_topic", map_topic, "/tb1/map"); 
	ros::param::param<std::string>(ns+"/robot_frame", frame_topic, "/tb1/base_link"); 

	ros::Subscriber map_sub = n.subscribe(map_topic, 50, mapCallBack);
	ros::Subscriber rviz_sub = n.subscribe("/clicked_point", 10, rvizCallBack);

	ros::Publisher detect_pub = n.advertise<geometry_msgs::PointStamped>("/detected_points", 10);
	ros::Publisher viz_pub = n.advertise<visualization_msgs::Marker>(ns+"/shapes", 10);

	ros::Rate rate(100);

	while(mapData.data.size() < 1 or mapData.header.seq < 1) {
		ros::spinOnce();
		rate.sleep();
	}

	// Rviz GUI Elements
	lines.header.frame_id = mapData.header.frame_id;
	points.header.frame_id = mapData.header.frame_id;
	points.header.stamp = ros::Time::now();
	lines.header.stamp = ros::Time::now();

	lines.ns = points.ns = "local_rrt";
	points.id = 0;
	lines.id = 1;

	points.action = points.ADD;
	lines.action = lines.ADD;
	points.pose.orientation.w = lines.pose.orientation.w = 1.0;
	lines.scale.x = lines.scale.y = 0.025;
	points.scale.x = points.scale.y = 0.25; 

	lines.color.r = 255.0/255.0;
	lines.color.g = 0.0/255.0;
	lines.color.b = 0.0/255.0;
	points.color.r = 255.0/255.0;
	points.color.g = 0.0/255.0;
	points.color.b = 0.0/255.0;
	points.color.a = 0.25;
	lines.color.a = 1.0;
	points.lifetime = lines.lifetime = ros::Duration();

	geometry_msgs::Point pnt;

	while(points.points.size() < 5) {
		ros::spinOnce();
		viz_pub.publish(points);
	}

	// Bounding Box and starting point
	std::vector<float> bounds1;
	bounds1.push_back(points.points[0].x);
	bounds1.push_back(points.points[0].y);

	std::vector<float> bounds2;
	bounds2.push_back(points.points[2].x);
	bounds2.push_back(points.points[0].y);

	init_x = Norm(bounds1, bounds2);
	bounds1.clear();
	bounds2.clear();

	bounds1.push_back(points.points[0].x);
	bounds1.push_back(points.points[0].y);

	bounds2.push_back(points.points[0].x);
	bounds2.push_back(points.points[2].y);

	init_y = Norm(bounds1, bounds2);
	bounds1.clear();
	bounds2.clear();

	startx = (points.points[0].x + points.points[2].x)/2;
	starty = (points.points[0].y + points.points[2].y)/2;

	geometry_msgs::Point start;
	start = points.points[4];
	std::vector<float> start_vec;
	start_vec.push_back(start.x);
	start_vec.push_back(start.y);
	std::vector<std::vector<float>> tree;
	tree.push_back(start_vec);
	points.points.clear();
	viz_pub.publish(points);

	//Frontier Detection
	std::vector<float> frontiers;
	int i = 0;
	float x, y;
	std::vector<float> x_rand, x_nearest, x_new;
	tf::TransformListener listener;

	while(ros::ok()) {
		x_rand.clear();
		x = (drand() * init_x) - (init_x/2) + startx;
		y = (drand() * init_y) - (init_y/2) + starty;

		x_rand.push_back(x);
		x_rand.push_back(y);

		x_nearest = Nearest(tree, x_rand);

		x_new = Steer(x_nearest, x_rand, eta);

		// Check for obstacles
		char check = ObstacleFree(x_nearest, x_new, mapData);
		
		if(check == -1) {
			goal.header.stamp = ros::Time::now();
			goal.header.frame_id = mapData.header.frame_id;
			goal.point.x = x_new[0];
			goal.point.y = x_new[1];
			goal.point.z = 0.0;
			
			pnt.x = x_new[0];
			pnt.y = x_new[1];
			pnt.z = 0.0;

			points.points.push_back(pnt);
			viz_pub.publish(points);

			detect_pub.publish(goal);
			points.points.clear();
			tree.clear();

			tf::StampedTransform transform;
			int temp = 0;
			while(temp < 1) {
				try {
					listener.lookupTransform(map_topic, frame_topic, ros::Time(0), transform);
					temp = 1;
				}
				catch(tf::TransformException &ex) {
					temp = 0;
					ros::Duration(0.1).sleep();
				}
			}

			x_new[0] = transform.getOrigin().x();
			x_new[1] = transform.getOrigin().y();
			tree.push_back(x_new);
			lines.points.clear();
		}

		else if(check == 1) {
			tree.push_back(x_new);
			pnt.x = x_new[0];
			pnt.y = x_new[1];
			pnt.z = 0.0;
			lines.points.push_back(pnt);
			pnt.x = x_nearest[0];
			pnt.y = x_nearest[1];
			pnt.z = 0.0;
			lines.points.push_back(pnt);
		}

		viz_pub.publish(lines);

		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}