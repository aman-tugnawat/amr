#include <iostream>
#include "ros/ros.h"
#include "sim/sim.h"

using namespace std;

int
main( int argc, char* argv[] ){

	Sim sim;
	ros::init( argc, argv, "sim_node" );

	ros::NodeHandle node_handle;

	ros::Subscriber command_subscriber = node_handle.subscribe( "cmd_vel_mux/input/navi", 1, &Sim::handle_command, &sim );
    
    //Obstacle sub
    ros::Subscriber obstacles_subscriber = node_handle.subscribe( "obstacles", 1, &Sim::handle_obstacles, &sim );

	ros::Publisher odometry_publisher = node_handle.advertise< nav_msgs::Odometry >( "odom", 1, true );

    //obsebation publisher
    ros::Publisher observations_publisher = node_handle.advertise< perception::Observations >( "observations", 1, true );
    
    //landmark publisher
    ros::Publisher landmark_publisher = node_handle.advertise< perception::Landmarks >( "landmarks", 1, true);

    //scan publisher
    ros::Publisher scan_publisher = node_handle.advertise< sensor_msgs::LaserScan >( "scan", 1, true);

    double frequency = 10.0;

	ros::Rate timer( frequency );
    sleep(1);
    landmark_publisher.publish( sim.landmarks_msg() );
	
    while ( ros::ok() ){
		sim.step( 1.0/frequency );
		odometry_publisher.publish( sim.odometry_msg() );
        observations_publisher.publish( sim.observations_msg() );
        //landmark_publisher.publish( sim.landmarks_msg() );
        scan_publisher.publish ( sim.scan_msg() );
		ros::spinOnce();
		timer.sleep();
	}
	return 0;
}
