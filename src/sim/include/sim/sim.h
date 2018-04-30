#ifndef SIM_H
#define SIM_H

#include <Eigen/Dense>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Polygon.h"
#include "sensor_msgs/LaserScan.h"
#include "perception/Observations.h"
#include "perception/Landmarks.h"

class Sim {
	public:
		Sim();
		virtual ~Sim();

		void step ( const double& dt );
		void handle_command ( const geometry_msgs::Twist::ConstPtr& msg );
        //obstacle
        void handle_obstacles( const geometry_msgs::Polygon::ConstPtr& msg );
		nav_msgs::Odometry odometry_msg( void )const;

        //obersavations with noise
		perception::Observations observations_msg( void );
	    
        //landmarks
        perception::Landmarks landmarks_msg( void );

        sensor_msgs::LaserScan scan_msg( void )const;

        //relative to robot frame
        geometry_msgs::Point relative_position( const geometry_msgs::Pose2D& robot, const geometry_msgs::Pose2D& world );

        //mappping -Laser scan and Obstacle
        geometry_msgs::Polygon& obstacles( void ){ return _obstacles; }

	protected:
		Eigen::Vector3d _x;
		Eigen::Vector3d _u;
		double _alpha1;
		double _alpha2;
		double _alpha3;
		double _alpha4;
		double _alpha5;
		double _alpha6;
		double _t;
		unsigned _num_scan_angles;
		unsigned _num_scan_distances;
        
        perception::Landmarks _landmarks;
        geometry_msgs::Polygon _obstacles;

};

#endif /* SIM_H */
