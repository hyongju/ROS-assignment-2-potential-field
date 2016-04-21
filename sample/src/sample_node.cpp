// A sample C++ code for assignment #2 
// This is a simple 2D motion planning program using potential field

#define ARRAY_SIZE(array) (sizeof((array))/sizeof((array[0])))
#define MAX_ARRAY_SIZE	100
#define LARGE_NUMBER	10000

#include <ros/ros.h>

#include <iostream>     			// to use: std::cout, std::cin, etc
#include <cmath>					// to use: atan2, sqrt, pow, etc
#include <iomanip>     				// to use: setprecision()
#include <geometry_msgs/Twist.h> 	
#include <nav_msgs/Odometry.h>
#include <vector>

// ROS/tf libraries
#include <tf/transform_listener.h>	// to use: tf listner
#include <tf/transform_broadcaster.h>	// to use: tf broadcaster
#include <tf/transform_datatypes.h>	// to use: tf datatypes

// Boost libraries (C++)
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/strategies/transform.hpp>

//============================================
// Uncomment this line for Boost version 1.5x
#include <boost/geometry/io/wkt/wkt.hpp>
//============================================
//============================================
// Uncomment this line for Boost version 1.4x
// #include <boost/geometry/io/wkt/wkt.hpp>
//============================================


// some type definitions to shorten the code
typedef boost::geometry::model::d2::point_xy<double> point_type;
typedef boost::geometry::model::polygon<point_type> poly_type;
typedef boost::geometry::model::linestring<point_type> linestring_type;


// a few empty string variables for later use...
std::string base_footprint;
std::string cmd_vel;
std::string odom;


// "motionPlanning" class
class motionPlanning
{
	// public functions, variables
	public:
		// constructor
		motionPlanning(ros::NodeHandle& n);
		
		// destructor
		~motionPlanning()
		{
		}
		// callback function invoked when a message is received from "odom" topic
		void odomCallback(const nav_msgs::Odometry& msg);
		
		// set youbot pose and transformation
		void setYoubot();
		
		// set obstacles positions and transformation
		void setObstacle();
		
		// transformation from "odom" to "base_footprint" (world -> robot coordinate)
		tf::Stamped<tf::Pose> odomToBase(const double & gx,const double & gy, const tf::TransformListener& listener);
		
		// get youbot current pose
		void getYoubotPose();
		
		// compute of configuration force using potential function based method
		geometry_msgs::Twist potentialField(const double & eta1, const double & eta2, const double & d_star, const double & Q_star);
		
		// obtain minimum distance between a control point on robot and each obstacle
		std::vector<double> minObstacleDistance(const point_type &pnt, const poly_type &poly);		
		
	private:

		// create a subscriber object;
        ros::Subscriber sub;
		
		// create a publisher object;	
        ros::Publisher pub;

		// create a transfor listener object
        tf::TransformListener listener;
		
		// pose of youbot's origin w.r.t. "odom" coordinate 
		double tf_yb_origin_x, tf_yb_origin_y, tf_yb_origin_yaw;
		
		// goal position (x,y) with respect to "odom" frame 
		double goal_x, goal_y;
		
		// goal position with respect to "base_footprint" frame
		double tf_goal_x, tf_goal_y;

		// youbot's corners points x,y as vectors
		std::vector<double> yb_corner_x;
		std::vector<double> yb_corner_y;
			
		// create polygon object with name "youbot"
		poly_type youbot;
		
		// at this moment, the number of obstacles should be specified (this will be fixed later)
		static const unsigned int NUMBER_OF_OBSTACLES = 5;		
		
		// create an empty polygon object for transformed obstacle
		poly_type tf_obs[NUMBER_OF_OBSTACLES];	

		//============================================================
		// define empty poses stamped to a specific coordinate system
		
		// youbot's origin w.r.t. "base_footprint"
		tf::Stamped<tf::Pose> yb_origin;
		
		// youbot's control points w.r.t. "base_footprint"
		tf::Stamped<tf::Pose> yb_corner[MAX_ARRAY_SIZE];
		
		// youbot's origin w.r.t. "odom"
		tf::Stamped<tf::Pose> tf_yb_origin;
		
		// youbot's origin w.r.t. "odom"
		tf::Stamped<tf::Pose> tf_yb_corner[MAX_ARRAY_SIZE];	        
};

motionPlanning::motionPlanning(ros::NodeHandle& n)
{
	geometry_msgs::Twist vel;		
	
	// error tolerance
	const double tol = 0.08;
	
	// constants, gains to be used in potential functions
	const double eta1 = -0.1, eta2 = -0.01;
	const double d_star = 1.0, Q_star = 0.2;
	
	// assigned to "pub" and advertise that we are going to publish msgs to cmd_vel with queue size of 10
	pub = n.advertise<geometry_msgs::Twist>(cmd_vel,10);
	
	// assign to "sub" and subsribe to "odom" topic via "odomCallback" function
	sub = n.subscribe(odom, 100, &motionPlanning::odomCallback,this);
	
	// define rate in ros compatible way
	ros::Rate loop_rate(10);
	
	// set youbot pose, transformation
	setYoubot();
	
	// set obstacles positions
	setObstacle();
	
	// get user input for goal positions in (x,y)
	std::cout << "Enter your initial goal: ";
	std::cin >> goal_x >> goal_y;
	
	while(n.ok()){
		// listening to messages
		ros::spinOnce();	
		
		// get youbot's position (which was received by the listener)
		getYoubotPose();

		
		// if robot is close enough to desired goal, do
		if (std::abs(goal_x - tf_yb_origin_x) < tol && std::abs(goal_y - tf_yb_origin_y) < tol)
		{	
			// stop the robot
			vel.linear.x = 0.0;
			vel.linear.y = 0.0;
			vel.angular.z = 0.0;		
			pub.publish(vel);
			
			// ask for another user input for next goal
			std::cout << "Enter your new goal: ";
			std::cin >> goal_x >> goal_y;
		}
		
		// obtain the goal position stamped to "base_foorprint"
		tf::Stamped<tf::Pose> tf_goal_pose = odomToBase(goal_x,goal_y,listener);

		tf_goal_x = tf_goal_pose.getOrigin().x();
		tf_goal_y = tf_goal_pose.getOrigin().y();	
		
		// display error to goal
		std::cout << "error to goal (dx,dy): (" << std::setprecision(3) << goal_x - tf_yb_origin_x << "," << std::setprecision(3) << goal_y - tf_yb_origin_y << ")"<< std::endl;
		
		// obtain total configuration force as the control input
		vel = potentialField(eta1, eta2, d_star, Q_star);
		
		// publish data to "cmd_vel" topic
		pub.publish(vel);
		
		// sleep at the defined rate (20Hz)
		loop_rate.sleep();
	}
}
void motionPlanning::getYoubotPose()
{
	// read robot pose including control points and origin w.r.t. "odom" coordinate frame
	for (unsigned i = 0; i != youbot.outer().size()-1; ++i) {
		listener.transformPose(odom,yb_corner[i], tf_yb_corner[i]);
		yb_corner_x.push_back(yb_corner[i].getOrigin().x());
		yb_corner_y.push_back(yb_corner[i].getOrigin().y()); // robot's corner positions in world coordinate frame
	}
	
	// transform youbot's origin pose from "base_foorprint" to "odom" coordinate
	listener.transformPose(odom, yb_origin, tf_yb_origin); 
	
	// assign to variables for conveniences
	tf_yb_origin_x = tf_yb_origin.getOrigin().x();
	tf_yb_origin_y = tf_yb_origin.getOrigin().y();			
	tf_yb_origin_yaw = tf::getYaw(tf_yb_origin.getRotation()); 
}

// This is the callback function for "odom" topic
void motionPlanning::odomCallback(const nav_msgs::Odometry& msg)
{
	// create an empty tf transformation object "pose_base"
	tf::Pose pose_base;							
	
	// change pose msg from topic "odom" to tf transformation and store to "pose_base"
	tf::poseMsgToTF(msg.pose.pose, pose_base);	
	
	// create a broadcaster  "br"
	tf::TransformBroadcaster br;

	// stamp the trasformation to be from "odom" to "base_footprint" and send this to broadcaster
	br.sendTransform(tf::StampedTransform(pose_base,ros::Time::now(),odom,base_footprint));
}


// This function sets pose, control points, and transformation of youbot
void motionPlanning::setYoubot()
{
	// polygon data for youbot
	boost::geometry::read_wkt("POLYGON((-0.3 0.2, 0.3 0.2, 0.3 -0.2, -0.3 -0.2, -0.3 0.2))",youbot);
	
	// define the pose of the origin
	yb_origin.frame_id_ = base_footprint;
	yb_origin.stamp_ = ros::Time(0);
	yb_origin.setData(tf::Pose(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0.0)));
	
	// define control points on the youbot
	for (unsigned i = 0; i != youbot.outer().size()-1; ++i) 
	{
		// stamped frame: "base_footprint"
		yb_corner[i].frame_id_ = base_footprint;
		
		// stamped time: closest available time
		yb_corner[i].stamp_ = ros::Time(0);
		
		// set pose data for each corner
		yb_corner[i].setData(tf::Pose(tf::Quaternion(0,0,0,1), tf::Vector3(boost::geometry::get<0>(youbot.outer()[i]),boost::geometry::get<1>(youbot.outer()[i]),0.0)));			
		
	}
}

// This function defines obstacles, and translate from obstacle to "odom" frame
// here, we only consider translation of the obstacles (no rotation)
void motionPlanning::setObstacle()
{
	// create an empty polygon object for obstacle
	poly_type obs[NUMBER_OF_OBSTACLES];
	
	// polygon data for obstacles w.r.t. obstacle coordinate frame
	boost::geometry::read_wkt("POLYGON((-3.5 3.5, -3.5 -3.5, 3.5 -3.5, 3.5 3.5, -3.5 3.5))", obs[0]);
	boost::geometry::read_wkt("POLYGON((-0.5 0.5, 0.5 0.5, 0.5 -0.5, -0.5 -0.5, -0.5 0.5))", obs[1]);    
	boost::geometry::read_wkt("POLYGON((-0.375 0.375, 0.375 0.375, 0.375 -0.375, -0.375 -0.375, -0.375 0.375))", obs[2]);    
	boost::geometry::read_wkt("POLYGON((-0.25 0.25, 0.25 0.25, 0.25 -0.25, -0.25 -0.25, -0.25 0.25))", obs[3]);    
	boost::geometry::read_wkt("POLYGON((-0.125 0.125, 0.125 0.125, 0.125 -0.125, -0.125 -0.125, -0.125 0.125))", obs[4]);    

	// translation data to move obstacles to
	double trans[][2] = {{2.5,2.5},{2.5,2.5},{5,1},{3,5},{0,3.0}};

	// tranlate each obstacle
	for (unsigned i = 0; i != ARRAY_SIZE(obs); ++i) {
		boost::geometry::strategy::transform::translate_transformer<point_type, point_type> translate(trans[i][0],trans[i][1]);
		boost::geometry::transform(obs[i], tf_obs[i], translate);
    }
}

// This function transforms a point from "odom" to "base_footprint" coordinate system
tf::Stamped<tf::Pose> motionPlanning::odomToBase(const double & gx,const double & gy, const tf::TransformListener& listener)
{
	// declare a "goal_pose" object stamp it to 'odom' coordinate system (global), and assign pose, at the most recent possible time.
	tf::Stamped<tf::Pose> pose_odom(
		tf::Pose(tf::Quaternion(0,0,0,1), tf::Vector3(gx,gy,0.0)),ros::Time(0), odom);
	
	// declare an empty stamped object named "tf_goal_pose"
	tf::Stamped<tf::Pose> pose_base;
	
	// transform "goal_pose" from "odom" to "base_footprint" coordinate frame and assign the resulting pose to "tf_goal_pose"
	listener.transformPose(base_footprint,pose_odom, pose_base);
	
	// return it
	return pose_base;
}

// This function computes the total configuration space force using potential function-based approach
geometry_msgs::Twist motionPlanning::potentialField(const double & eta1, const double & eta2, const double & d_star, const double & Q_star)
{
	// declare local variables here
	double d_u_att_x, d_u_att_y, d_u_att_t;
	double d_u_rep_x, d_u_rep_y, d_u_rep_t;
	geometry_msgs::Twist vel;
	
	// iterate through each control point on youbot
	for (unsigned i = 0; i != youbot.outer().size()-1; ++i) 
	{
		//===================================
		// attractive potential
		//===================================
		
		// distance between i-th control point and the target point (goal)
		double d_corner_to_goal = boost::geometry::distance(point_type(tf_yb_corner[i].getOrigin().x(), tf_yb_corner[i].getOrigin().y()), point_type(goal_x, goal_y));

		if (d_corner_to_goal <= d_star)
		{
			d_u_att_x = eta1 * (yb_corner_x[i] - tf_goal_x);
			d_u_att_y = eta1 * (yb_corner_y[i] - tf_goal_y);
		
		}
		else
		{
			d_u_att_x = d_star * eta1 * (yb_corner_x[i] - tf_goal_x) / d_corner_to_goal;
			d_u_att_y = d_star * eta1 * (yb_corner_y[i] - tf_goal_y) / d_corner_to_goal;
		}
		d_u_att_t = -d_u_att_x *(yb_corner_x[i]*sin(tf_yb_origin_yaw)+yb_corner_y[i]*cos(tf_yb_origin_yaw)) + d_u_att_y * (yb_corner_x[i]*cos(tf_yb_origin_yaw)-yb_corner_y[i]*sin(tf_yb_origin_yaw));
	
		//===================================
		// Repulsive potential
		//===================================
	
	
		for (unsigned j = 0; j != NUMBER_OF_OBSTACLES; ++j) 
		{
			// the miminum distance between i-th control point and j-th obstacle, with coordinate of points on the obstacle that makes it minimum
			std::vector<double> vt = minObstacleDistance(point_type(tf_yb_corner[i].getOrigin().x(), tf_yb_corner[i].getOrigin().y()),tf_obs[j]);
			double d_min_to_obs = vt[0];
			double min_pnt_on_obs_x = vt[1];
			double min_pnt_on_obs_y = vt[2];
		
			// Transformation of the point on each of the closest obstacle from "odom" to "base_footprint" coordinate
			tf::Stamped<tf::Pose> tf_obs_pose = odomToBase(min_pnt_on_obs_x,min_pnt_on_obs_y,listener);
			
			double tf_obs_pose_x = tf_obs_pose.getOrigin().x();
			double tf_obs_pose_y = tf_obs_pose.getOrigin().y();
			
			if (d_min_to_obs <= Q_star)
			{
				d_u_rep_x = eta2 * (1/Q_star - 1/ d_min_to_obs) / pow(d_min_to_obs,2) * (yb_corner_x[i]-tf_obs_pose_x); 
				d_u_rep_y =	eta2 * (1/Q_star - 1/ d_min_to_obs) / pow(d_min_to_obs,2) * (yb_corner_y[i]-tf_obs_pose_y);
			}
			else
			{
				d_u_rep_x = 0.0;
				d_u_rep_y = 0.0;
			}
			d_u_rep_t = -d_u_rep_x *(yb_corner_x[i]*sin(tf_yb_origin_yaw)+yb_corner_y[i]*cos(tf_yb_origin_yaw)) + d_u_rep_y * (yb_corner_x[i]*cos(tf_yb_origin_yaw)-yb_corner_y[i]*sin(tf_yb_origin_yaw));
			
			// summation over all forces acting on each coordinate, i.e., sum_i sum_j
			vel.linear.x += d_u_rep_x;
			vel.linear.y += d_u_rep_y;
			//vel.angular.z += d_u_rep_t;

		}			
	
		// Summation of all forces acting on in each coordinate, i.e., sum_i
		vel.linear.x += (d_u_att_x);
		vel.linear.y += (d_u_att_y);
		//vel.angular.z += (d_u_att_t);
	}
	
	// return control input "vel"
	return vel;
}


// This function calculates the distance between point and a polygon obstacle
// =====================================================================
// INPUT: point "pnt" (datatype: point_type), polygon "poly" (datatype: poly_type)
// OUTPUT: the closest point "(ver_x, ver_y)", the minimal distance "d_min" in a vector form
// =====================================================================

std::vector<double> motionPlanning::minObstacleDistance(const point_type &pnt, const poly_type &poly)
{
	// some declaration of variables and initialization
	double x[poly.outer().size()];	
	double y[poly.outer().size()];
	double d_min = LARGE_NUMBER;
	std::vector<double> output;
	double ver_x, ver_y;
	double dist_min[poly.outer().size()-1];
	
	// find the minimum distance between a point and the vertices from the polygon and the closest vertex
	for (unsigned i = 0; i != poly.outer().size(); ++i) {
		
		// this extracts all the vertices form the polygon
		x[i] = boost::geometry::get<0>(poly.outer()[i]);
		y[i] = boost::geometry::get<1>(poly.outer()[i]);
	
		// find the minimum between "pnt" and the vertex "(x[i],y[i])"
		double d_min_tmp = boost::geometry::distance(pnt,point_type(x[i],y[i]));
		if (d_min_tmp <= d_min)
		{
			d_min = d_min_tmp;
			ver_x = x[i];
			ver_y = y[i];
		}
		// "(ver_x, ver_y)" is the vertex with minimal distance to "pnt"
	}
	
	// This for loop computes the minimum among distances between "pnt" and all edges of "poly"
	// if the distance is closer than the closest vertex, it means that the closest point is 
	// not one of the vertices but a point lies on the edges
	// this function computes the point, and the minimum distance
	for (unsigned i = 0; i != poly.outer().size()-1; ++i) {
		// create a line object "line"
		
		linestring_type line;						
		
		// add points to "line"
		
		line.push_back(point_type(x[i],y[i]));		
		line.push_back(point_type(x[i+1],y[i+1]));	
		
		// compute distance between point "pnt" and "line"
		
		dist_min[i] = boost::geometry::distance(pnt, line); 
		
		// if the new distance is smaller than what we have,
		// it means that the closest point is not one of the vertices
		// compute the point and update "d_min"
		
		if (dist_min[i] < d_min)
		{
			// we are going to check through all edges (CW direction)
			
			point_type v1(x[i],y[i]);		// 1st vertex
			point_type v2(x[i+1],y[i+1]);	// 2nd vertex
			
			// distance between "pnt" and the 1st vertex
			double d_pnt_v1 = boost::geometry::distance(pnt, v1);
			
			// distance between "pnt" and the 2nd vertex
			double d_pnt_v2 = boost::geometry::distance(pnt, v2);
			
			// distance between the 1st and 2nd vertex
			double d_v1_v2 = boost::geometry::distance(v1, v2);
			
			// distance between minimum point (on the polygon edge) and and the 1st vertex 
			// using "Pythagorean theorem"
			double d_v1_x = sqrt(pow(d_pnt_v1,2) - pow(dist_min[i],2));
			
			// find the minimum point (ver_x,ver_y) on the polygon edge and the minimal distance between "pnt" and minimum (ver_x,ver_y)
			ver_x = d_v1_x * (x[i+1] - x[i])/d_v1_v2 + x[i];
			ver_y = d_v1_x * (y[i+1] - y[i])/d_v1_v2 + y[i];
			d_min = dist_min[i];
		}
	}
	// output vector
	output.push_back(d_min);
	output.push_back(ver_x);
	output.push_back(ver_y);
	return output;
}

int main(int argc, char **argv)
{
	// assign proper names for robot base, command velocity, and odom topic
	base_footprint = "base_footprint";		// a name for robot's coordinate
	odom = "odom";							// a name for world's coordinate
	cmd_vel = "cmd_vel";					

	// initialize as a ROS program
    ros::init(argc, argv,"sample_node",ros::init_options::NoSigintHandler);  
    
    // create node handle
    ros::NodeHandle n;	
    
    // create motionPlanning object with the node handler
    motionPlanning mp(n);
    
	return 0;
}








