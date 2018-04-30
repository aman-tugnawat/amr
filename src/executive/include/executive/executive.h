#ifndef EXECUTIVE_H
#define EXECUTIVE_H

#include <iostream>
#include "ros/ros.h"

#include <iostream>
#include <vector>
#include "geometry_msgs/Point32.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"

using namespace std;

class Waypoints {
    public:        
        Waypoints() {};
        virtual ~Waypoints(){};
        vector<geometry_msgs::Point32> points;
        
};

class Executive{
	public:
		Executive();
		virtual ~Executive();
		void handleOdom( const nav_msgs::Odometry::ConstPtr& msg );
        void handle_path( const nav_msgs::Path::ConstPtr& msg );// new function callback
        ros::Publisher goal_publisher;
        //Waypoints waypoints;
        //nav_msgs::Odometry _odom;
    private:        
        int i;
    protected:
        Waypoints waypoints;
        nav_msgs::Odometry _odom;
        double _distance_threshold;
        double _replan_distance_threshold;// new threshold
        ros::Publisher _goal_publisher;
        
        
};

#endif /* EXECUTIVE_H */
		
