#include <iostream>
#include "ros/ros.h"
#include "perception/Landmarks.h"
using namespace std;
int main( int argc , char* argv[] ){
	ros::init( argc, argv, "landmarks_publisher" );
	ros::NodeHandle node_handle;
	ros::Publisher goal_publisher = node_handle.advertise< perception::Landmarks >( "landmarks", 1, true);
	sleep( 5 );
	cout << "creating landmarks" << endl;
    perception::Landmarks landmarks;

        landmarks.landmarks.push_back( perception::Landmark() );
    landmarks.landmarks.back().pos.x = -3.49156;
    landmarks.landmarks.back().pos.y = 1.12066;
    landmarks.landmarks.back().signature = 4;

    landmarks.landmarks.push_back( perception::Landmark() );
    landmarks.landmarks.back().pos.x = -10.4462;
    landmarks.landmarks.back().pos.y = 2.03502;
    landmarks.landmarks.back().signature = 24;

    landmarks.landmarks.push_back( perception::Landmark() );
    landmarks.landmarks.back().pos.x = -4.57531;
    landmarks.landmarks.back().pos.y = 2.70068;
    landmarks.landmarks.back().signature = 25;

    landmarks.landmarks.push_back( perception::Landmark() );
    landmarks.landmarks.back().pos.x = -6.27467;
    landmarks.landmarks.back().pos.y = 1.32434;
    landmarks.landmarks.back().signature = 26;

    landmarks.landmarks.push_back( perception::Landmark() );
    landmarks.landmarks.back().pos.x = -8.48433;
    landmarks.landmarks.back().pos.y = 1.14299;
    landmarks.landmarks.back().signature = 27;

    landmarks.landmarks.push_back( perception::Landmark() );
    landmarks.landmarks.back().pos.x = -10.6895;
    landmarks.landmarks.back().pos.y = -1.13448;
    landmarks.landmarks.back().signature = 28;

    landmarks.landmarks.push_back( perception::Landmark() );
    landmarks.landmarks.back().pos.x = -8.53808;
    landmarks.landmarks.back().pos.y = 2.93891;
    landmarks.landmarks.back().signature = 29;

    landmarks.landmarks.push_back( perception::Landmark() );
    landmarks.landmarks.back().pos.x = -1.87158;
    landmarks.landmarks.back().pos.y = 0.910797;
    landmarks.landmarks.back().signature = 30;

    landmarks.landmarks.push_back( perception::Landmark() );
    landmarks.landmarks.back().pos.x = -1.19871;
    landmarks.landmarks.back().pos.y = 2.37738;
    landmarks.landmarks.back().signature = 31;

    landmarks.landmarks.push_back( perception::Landmark() );
    landmarks.landmarks.back().pos.x = -0.796101;
    landmarks.landmarks.back().pos.y = -0.260302;
    landmarks.landmarks.back().signature = 32;

    landmarks.landmarks.push_back( perception::Landmark() );
    landmarks.landmarks.back().pos.x = -9.53858;
    landmarks.landmarks.back().pos.y = 3.08835;
    landmarks.landmarks.back().signature = 33;

    landmarks.landmarks.push_back( perception::Landmark() );
    landmarks.landmarks.back().pos.x = -1.90964;
    landmarks.landmarks.back().pos.y = 2.48931;
    landmarks.landmarks.back().signature = 34;

    landmarks.landmarks.push_back( perception::Landmark() );
    landmarks.landmarks.back().pos.x = -8.89582;
    landmarks.landmarks.back().pos.y = -1.30229;
    landmarks.landmarks.back().signature = 35;

    landmarks.landmarks.push_back( perception::Landmark() );
    landmarks.landmarks.back().pos.x = -9.43398;
    landmarks.landmarks.back().pos.y = -2.58959;
    landmarks.landmarks.back().signature = 36;

    landmarks.landmarks.push_back( perception::Landmark() );
    landmarks.landmarks.back().pos.x = -7.53933;
    landmarks.landmarks.back().pos.y = 1.43559;
    landmarks.landmarks.back().signature = 37;

    landmarks.landmarks.push_back( perception::Landmark() );
    landmarks.landmarks.back().pos.x = 0.836418;
    landmarks.landmarks.back().pos.y = 0.0475866;
    landmarks.landmarks.back().signature = 38;

    landmarks.landmarks.push_back( perception::Landmark() );
    landmarks.landmarks.back().pos.x = 0.605168;
    landmarks.landmarks.back().pos.y = -2.16017;
    landmarks.landmarks.back().signature = 39;

    landmarks.landmarks.push_back( perception::Landmark() );
    landmarks.landmarks.back().pos.x = -10.6307;
    landmarks.landmarks.back().pos.y = -0.290646;
    landmarks.landmarks.back().signature = 41;

    landmarks.landmarks.push_back( perception::Landmark() );
    landmarks.landmarks.back().pos.x = -10.704;
    landmarks.landmarks.back().pos.y = -1.39038;
    landmarks.landmarks.back().signature = 42;

    landmarks.landmarks.push_back( perception::Landmark() );
    landmarks.landmarks.back().pos.x = 0.704546;
    landmarks.landmarks.back().pos.y = -0.804257;
    landmarks.landmarks.back().signature = 43;

    landmarks.landmarks.push_back( perception::Landmark() );
    landmarks.landmarks.back().pos.x = -0.939096;
    landmarks.landmarks.back().pos.y = -3.24702;
    landmarks.landmarks.back().signature = 44;

    landmarks.landmarks.push_back( perception::Landmark() );
    landmarks.landmarks.back().pos.x = -0.975399;
    landmarks.landmarks.back().pos.y = -1.78767;
    landmarks.landmarks.back().signature = 45;

    landmarks.landmarks.push_back( perception::Landmark() );
    landmarks.landmarks.back().pos.x = 0.820696;
    landmarks.landmarks.back().pos.y = -0.167103;
    landmarks.landmarks.back().signature = 46;

    landmarks.landmarks.push_back( perception::Landmark() );
    landmarks.landmarks.back().pos.x = -10.4122;
    landmarks.landmarks.back().pos.y = 2.30557;
    landmarks.landmarks.back().signature = 47;

    landmarks.landmarks.push_back( perception::Landmark() );
    landmarks.landmarks.back().pos.x = 1.02864;
    landmarks.landmarks.back().pos.y = 1.28695;
    landmarks.landmarks.back().signature = 48;

    cout << "landmarks:" << endl << landmarks;
	cout << "publishing landmarks" << endl;
	goal_publisher.publish( landmarks );
	sleep(1);
	cout << "done" << endl;
	return 0;
}
