#include <iostream>
#include "ros/ros.h"
#include "perception/perception.h"
using namespace std;
int main( int argc, char* argv[] ){
    Perception perception;
    cout << "perception:(" << perception << ")" << endl;
    ros::init( argc, argv, "perception_node" );
    ros::NodeHandle node_handle;
    ros::Subscriber rgb_image_subscriber = node_handle.subscribe( "camera/rgb/image_raw", 1, &Perception::handleRGBImage, &perception );
    ros::Subscriber depth_image_subscriber = node_handle.subscribe( "camera/depth/image_raw", 1, &Perception::handleDepthImage, &perception );
    
    //obsebation publisher
    ros::Publisher observations_publisher = node_handle.advertise< perception::Observations >( "observations", 1, true );
    
    //landmark publisher
    ros::Publisher landmark_publisher = node_handle.advertise< perception::Landmarks >( "landmarks", 1, true);    

    sleep( 1 );
    landmark_publisher.publish( perception.landmarks_msg() );
    double frequency = 10.0; ros::Rate timer( frequency );
    while ( ros::ok() ){
        ros::spinOnce();
        observations_publisher.publish( perception.observations );
        timer.sleep();
    }
    return 0;
}
