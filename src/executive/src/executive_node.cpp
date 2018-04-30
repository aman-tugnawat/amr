#include <iostream>
#include "ros/ros.h"
#include "executive/executive.h"

using namespace std;

int main( int argc, char* argv[] ){

	Executive exe;
	ros::init( argc, argv, "exective_node" );

	ros::NodeHandle node_handle;

	ros::Subscriber subscriber_odom = node_handle.subscribe( "odom", 1, &Executive::handleOdom, &exe );
    //ros::Subscriber subscriber_odom = node_handle.subscribe( "estimated_odom", 1, &Executive::handleOdom, &exe );
    //goal publisher
	exe.goal_publisher = node_handle.advertise< geometry_msgs::Pose >( "goal", 1, true);
    //ros::Subscriber waypoints_subscriber = node_handle.subscribe( "waypoints",1, &Executive::handle_waypoints, &executive );

    ros::Subscriber path_subscriber = node_handle.subscribe( "path", 1, &Executive::handle_path, &exe );
    // new subscriber
    
    double frequency = 10.0;
    ros::Rate timer( frequency );
    while( ros::ok() ){
        ros::spinOnce();
        timer.sleep();
    }
    
	return 0;
}
