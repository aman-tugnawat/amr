#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
using namespace std;
int 
main( int argc, char* argv[] ){
	ros::init( argc, argv, "sim_go" );
	ros::NodeHandle node_handle;
	ros::Publisher command_publisher = node_handle.advertise< geometry_msgs::Twist >( "cmd_vel_mux/input/navi", 1, true );
	geometry_msgs::Twist msg;
	msg.linear.x = 0.5;
	msg.angular.z = 0.5;

	command_publisher.publish( msg );
	sleep( 1 );
	return 0;
}
