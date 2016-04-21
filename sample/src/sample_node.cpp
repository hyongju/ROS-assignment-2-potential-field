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
#include <tf/transform_listener.h>		// to use: tf listner
#include <tf/transform_broadcaster.h>	// to use: tf broadcaster
#include <tf/transform_datatypes.h>		// to use: tf datatypes

// a few empty strings for later use...
std::string base_footprint;
std::string cmd_vel;
std::string odom;

struct Points2D {
 Points2D( double x, double y):_x(x),_y(y)
 {
 }
 double _x , _y;
};

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
		
		// set youbot pose and transformation
		void setYoubot();
		
		// set obstacles positions and transformation
		void setObstacle();
		
		// get youbot current pose
		void getYoubotPose();
		
		// distance between point and point
		double distPointPoint(std::vector<Points2D> v1, std::vector<Points2D> v2);

		// distance between point and line, and the closest point on the line
		std::vector<double> distPointLine(std::vector<Points2D> P, std::vector<Points2D> L);
		
		// dot product
		double dotP(std::vector<Points2D> v1, std::vector<Points2D> v2);

		// compute of configuration force using potential function based method
		geometry_msgs::Twist potentialField(const double & eta1, const double & eta2, const double & d_star, const double & Q_star);
		
	
		// distance between point and polygon, and the closest point on the polygon
		std::vector<double> distPointPolygon(std::vector<Points2D> pnt, std::vector<Points2D> poly);		
		
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
		
		// youbot's corners points x,y as vectors
		std::vector<double> yb_corner_x;
		std::vector<double> yb_corner_y;
			
		// create polygon object with name "youbot"
		std::vector<Points2D> youbot;
		
		// at this moment, the number of obstacles should be specified (this will be fixed later)
		static const unsigned int NUMBER_OF_OBSTACLES = 5;		
		
		// create an empty polygon object for transformed obstacle
		//poly_type tf_obs[NUMBER_OF_OBSTACLES];
		//std::vector<Points2D> tf_obs[NUMBER_OF_OBSTACLES];
		std::vector<std::vector<Points2D> > tf_obs;	

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
	const double tol = 0.05;
	
	// constants, gains to be used in potential functions
	const double eta1 = -0.1, eta2 = -0.01;
	const double d_star = 1.0, Q_star = 0.5;
	
	// assigned to "pub" and advertise that we are going to publish msgs to cmd_vel with queue size of 10
	pub = n.advertise<geometry_msgs::Twist>(cmd_vel,10);
	
	// define rate in ros compatible way
	ros::Rate loop_rate(25);
	
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
	for (unsigned i=0; i < youbot.size()-1; i++) {
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



// This function sets pose, control points, and transformation of youbot
void motionPlanning::setYoubot()
{
	youbot.push_back(Points2D(-0.3,0.2));	
	youbot.push_back(Points2D(0.0,0.2));	
	youbot.push_back(Points2D(0.3,0.2));	
	youbot.push_back(Points2D(0.3,0.0));	
	youbot.push_back(Points2D(0.3,-0.2));	
	youbot.push_back(Points2D(0.0,-0.2));	
	youbot.push_back(Points2D(-0.3,-0.2));	
	youbot.push_back(Points2D(-0.3,0.0));	
	youbot.push_back(Points2D(-0.3,0.2));	

	// define the pose of the origin
	yb_origin.frame_id_ = base_footprint;
	yb_origin.stamp_ = ros::Time(0);
	yb_origin.setData(tf::Pose(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0.0)));

	for (unsigned i=0; i < youbot.size()-1; i++)
	{
		// stamped frame: "base_footprint"
		yb_corner[i].frame_id_ = base_footprint;
		
		// stamped time: closest available time
		yb_corner[i].stamp_ = ros::Time(0);
		
		// set pose data for each corner
		yb_corner[i].setData(tf::Pose(tf::Quaternion(0,0,0,1), tf::Vector3(youbot[i]._x,youbot[i]._y,0.0)));			
	}
}

// This function defines obstacles, and translate from obstacle to "odom" frame
// here, we only consider translation of the obstacles (no rotation)
void motionPlanning::setObstacle()
{
	// square widths
	double cord[5] = {3.5, 0.5, 0.375, 0.25, 0.125};

	// translate obstacle to...
	double trans[][2] = {{2.5,2.5},{2.5,2.5},{5,1},{3,5},{0,3.0}};
	
	for (unsigned i = 0; i!= ARRAY_SIZE(cord)-1; ++i)
	{
		std::vector<Points2D> row;
		row.push_back(Points2D(-cord[i]+trans[i][0],cord[i]+trans[i][1]));	
		row.push_back(Points2D(cord[i]+trans[i][0],cord[i]+trans[i][1]));	
		row.push_back(Points2D(cord[i]+trans[i][0],-cord[i]+trans[i][1]));	
		row.push_back(Points2D(-cord[i]+trans[i][0],-cord[i]+trans[i][1]));	
		row.push_back(Points2D(-cord[i]+trans[i][0],cord[i]+trans[i][1]));	
		tf_obs.push_back(row);
	}
}

// This function computes the total configuration space force using potential function-based approach
geometry_msgs::Twist motionPlanning::potentialField(const double & eta1, const double & eta2, const double & d_star, const double & Q_star)
{
	// declare local variables here
	double d_u_att_x, d_u_att_y, d_u_att_t;
	double d_u_rep_x, d_u_rep_y, d_u_rep_t;
	geometry_msgs::Twist vel;
	
	// iterate through each control point on youbot
	for (unsigned i=0; i < youbot.size()-1; i++) 
	{
		//===================================
		// attractive potential
		//===================================
		
		// distance between i-th control point and the target point (goal)

		std::vector<Points2D> l1;
		std::vector<Points2D> l2;
		l1.push_back(Points2D(tf_yb_corner[i].getOrigin().x(), tf_yb_corner[i].getOrigin().y()));
		l2.push_back(Points2D(goal_x, goal_y));
		double d_corner_to_goal = distPointPoint(l1,l2);
		if (d_corner_to_goal <= d_star)
		{
			d_u_att_x = eta1 * (tf_yb_corner[i].getOrigin().x() - goal_x);
			d_u_att_y = eta1 * (tf_yb_corner[i].getOrigin().y() - goal_y);
		}
		else
		{
			d_u_att_x = d_star * eta1 * (tf_yb_corner[i].getOrigin().x() - goal_x) / d_corner_to_goal;
			d_u_att_y = d_star * eta1 * (tf_yb_corner[i].getOrigin().y() - goal_y) / d_corner_to_goal;
		}
		d_u_att_t = -d_u_att_x *(yb_corner[i].getOrigin().x()*sin(tf_yb_origin_yaw)+yb_corner[i].getOrigin().y()*cos(tf_yb_origin_yaw)) + d_u_att_y * (yb_corner[i].getOrigin().x()*cos(tf_yb_origin_yaw)-yb_corner[i].getOrigin().y()*sin(tf_yb_origin_yaw));
	
		//===================================
		// Repulsive potential
		//===================================
	
		for (unsigned j = 0; j != tf_obs.size()-1; ++j) 
		{
			// the miminum distance between i-th control point and j-th obstacle, with coordinate of points on the obstacle that makes it minimum
			std::vector<Points2D> l3; 
			l3.push_back(Points2D(tf_yb_corner[i].getOrigin().x(), tf_yb_corner[i].getOrigin().y()));
			std::vector<double> vt = distPointPolygon(l3,tf_obs[j]);

			double d_min_to_obs = vt[0];
			double min_pnt_on_obs_x = vt[1];
			double min_pnt_on_obs_y = vt[2];
			if (j == 1){
				std::cout << l3[0]._x << " " << l3[0]._y << std::endl;
				std::cout << d_min_to_obs << " " << min_pnt_on_obs_x << ", " << min_pnt_on_obs_y << std::endl;
			}
			
			if (d_min_to_obs <= Q_star)
			{
				d_u_rep_x = eta2 * (1/Q_star - 1/ d_min_to_obs) / pow(d_min_to_obs,2) * (tf_yb_corner[i].getOrigin().x()-min_pnt_on_obs_x); 
				d_u_rep_y = eta2 * (1/Q_star - 1/ d_min_to_obs) / pow(d_min_to_obs,2) * (tf_yb_corner[i].getOrigin().y()-min_pnt_on_obs_y);
			}
			else
			{
				d_u_rep_x = 0.0;
				d_u_rep_y = 0.0;
			}
			d_u_rep_t = -d_u_rep_x *(yb_corner[i].getOrigin().x()*sin(tf_yb_origin_yaw)+yb_corner[i].getOrigin().y()*cos(tf_yb_origin_yaw)) + d_u_rep_y * (yb_corner[i].getOrigin().x()*cos(tf_yb_origin_yaw)-yb_corner[i].getOrigin().y()*sin(tf_yb_origin_yaw));
			
			// summation over all forces acting on each coordinate, i.e., sum_i sum_j
			vel.linear.x += d_u_rep_x;
			vel.linear.y += d_u_rep_y;
			vel.angular.z += d_u_rep_t;

		}			
	
		// Summation of all forces acting on in each coordinate, i.e., sum_i
		vel.linear.x += (d_u_att_x);
		vel.linear.y += (d_u_att_y);
		vel.angular.z += (d_u_att_t);
	}
	
	// return control input "vel"
	return vel;
}


// This function calculates the distance between point and a polygon and also returns the closest point on the polygon
// =====================================================================
// INPUT: point "pnt" (datatype: Points2D), polygon "poly" (datatype: Points2D)
// OUTPUT: he minimal distance "output[0]", the closest point "(output[1], output[2])" in a vector form
// =====================================================================

std::vector<double> motionPlanning::distPointPolygon(std::vector<Points2D> pnt, std::vector<Points2D> poly)
{
	// some declaration of variables and initialization

	std::vector<double> output;
	std::vector<double> min_val;
	std::vector<Points2D> min_pnt;
	double min_val_so_far = LARGE_NUMBER;
	size_t min_idx_so_far;
		
	for (size_t i = 0; i != poly.size()-1; ++i){
		std::vector<Points2D> cur_line;
		std::vector<double> tmp_str;
		cur_line.push_back(Points2D(poly[i]._x, poly[i]._y));
		cur_line.push_back(Points2D(poly[i+1]._x, poly[i+1]._y));
		tmp_str = distPointLine(pnt, cur_line);
		
		min_val.push_back(tmp_str[0]);
		min_pnt.push_back(Points2D(tmp_str[1], tmp_str[2]));
		if (min_val[i] < min_val_so_far){
			min_val_so_far = min_val[i];
			min_idx_so_far = i;
		}
	}
	// output vector
	output.push_back(min_val[min_idx_so_far]);
	output.push_back(min_pnt[min_idx_so_far]._x);
	output.push_back(min_pnt[min_idx_so_far]._y);
	return output;
}


double motionPlanning::distPointPoint(std::vector<Points2D> v1, std::vector<Points2D> v2)
{
	return sqrt(pow(v1[0]._x-v2[0]._x,2) + pow(v1[0]._y-v2[0]._y,2));
}


std::vector<double> motionPlanning::distPointLine(std::vector<Points2D> P, std::vector<Points2D> L)
{
	std::vector<double> output;		
	std::vector<Points2D> P0;
	P0.push_back(Points2D(L[0]._x,L[0]._y));
	std::vector<Points2D> P1;
	P1.push_back(Points2D(L[1]._x,L[1]._y));
	std::vector<Points2D> v;
	v.push_back(Points2D(P1[0]._x - P0[0]._x, P1[0]._y - P0[0]._y));
	std::vector<Points2D> w;
	w.push_back(Points2D(P[0]._x - P0[0]._x, P[0]._y - P0[0]._y));
	std::vector<Points2D> v1;
	v1.push_back(Points2D(P0[0]._x - P1[0]._x, P0[0]._y - P1[0]._y));
	std::vector<Points2D> w1;
	w1.push_back(Points2D(P[0]._x - P1[0]._x, P[0]._y - P1[0]._y));	
    //v = P1 - P0
    //w = P - P0
	double c1 = dotP(v,w);
	double c2 = dotP(v1,w1);
	double c3 = dotP(v,v);
	if ( c1 <= 0.0 )  // before P0
	{
		output.push_back(distPointPoint(P, P0));
		output.push_back(P0[0]._x);
		output.push_back(P0[0]._y);
		return output;
	}
	if ( c2 <= 0.0 )  // before P1
	{
		output.push_back(distPointPoint(P, P1));
		output.push_back(P1[0]._x);
		output.push_back(P1[0]._y);		
		return output;
	}
    double b = c1/c3;
    std::vector<Points2D> Pb;
    Pb.push_back(Points2D(P0[0]._x + (v[0]._x) * b, P0[0]._y + (v[0]._y) * b));
	output.push_back(distPointPoint(P, Pb));
	output.push_back(Pb[0]._x);
	output.push_back(Pb[0]._y);	    
    return output;
}

double motionPlanning::dotP(std::vector<Points2D> v1, std::vector<Points2D> v2)
{
	return v1[0]._x * v2[0]._x + v1[0]._y * v2[0]._y;
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

 
