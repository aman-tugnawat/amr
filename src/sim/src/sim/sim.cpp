#include <iostream>
#include <math.h>
#include "sim/sim.h"

using namespace std;
/**
x = ( x_position y_position heading )
u = ( linear_velocity-(v) angluar_velocity-(w) )  

*/
geometry_msgs::Quaternion
yaw_to_quaternion( const double& yaw ){
	geometry_msgs::Quaternion quaternion;
	quaternion.w = cos( yaw / 2.0 );
	quaternion.x = 0.0;
	quaternion.y = 0.0;
	quaternion.z = sin( yaw /2.0 );
	return quaternion;
}

double
sample( const double& bsquared ){
	double tmp = 0.0;
    double b = 0.0;
    b = sqrt(bsquared);
	// fill this in 
    for(int i=0;i<12;i++){
        tmp += 0.5*(-b+2.0*b*(double)(rand()%1000)/(double)(1000));
    }
	return tmp;
}

Sim::
Sim() : _x( 0.0, 0.0, 0.0 ), _u( 0.0, 0.0, 0.0 ), _alpha1( 0.005 ), _alpha2( 0.05 ), _alpha3( 0.005 ), _alpha4( 0.05 ), _alpha5( 0.005 ), _alpha6( 0.05 ), _t( 0.0 ), _num_scan_angles( 128 ), _num_scan_distances( 256 ) {

}

Sim::
~Sim() {

}

void
Sim::
step( const double& dt ){
	Eigen::Vector3d uhat( _u );
	//fill this in
	uhat( 0 ) = _u( 0 ) + _alpha1*sample(_u(0)*_u(0))+_alpha2*sample(_u(1)*_u(1));
	uhat( 1 ) = _u( 1 ) + _alpha3*sample(_u(0)*_u(0))+_alpha4*sample(_u(1)*_u(1));
    uhat( 2 ) = _u( 2 );

	Eigen::Vector3d dx( 0.0, 0.0, 0.0 );
	//fill this in
    if(!uhat( 1 )==0.0){
	    dx( 0 ) = (-uhat(0)/uhat(1)*sin(_x(2))) + (uhat(0)/uhat(1)*sin(_x(2)+uhat(1)*dt));
	
	    dx( 1 ) = (uhat(0)/uhat(1)*cos(_x(2))) - (uhat(0)/uhat(1)*cos(_x(2)+uhat(1)*dt));

	    dx( 2 ) = uhat( 1 )*dt;
	}else {
        dx( 0 ) = uhat( 0 )*sin( _x(2) )*dt;
        dx( 1 ) = uhat( 0 )*cos( _x(2) )*dt;
        dx( 2 ) = 0.0;
    }
	
	

	_x += dx;
    //_x( 2 ) = _x( 2 )%(2.0*M_PI);
	_t += dt;

	cout << "_u[3]:{" << _u( 0 ) << " , " << _u( 1 ) << " , " << _u( 2 ) << " } " << endl;

	cout << "uhat[3]:{" << uhat( 0 ) << "," << uhat( 1 ) << "," << uhat( 2 ) << "}" << endl;

	cout << "_x[3]:{" << _x( 0 ) << "," << _x( 1 ) << "," << _x( 2 ) << "}" << endl;

	cout << "_t:" << _t << endl << endl;
	return;
}

void
Sim::
handle_command( const geometry_msgs::Twist::ConstPtr& msg ){
    _u( 0 ) = msg->linear.x;
    _u( 1 ) = msg->angular.z;
    return;
}

void
Sim::
handle_obstacles( const geometry_msgs::Polygon::ConstPtr& msg ){
    _obstacles = *msg;
    return;
}


nav_msgs::Odometry
Sim::
odometry_msg( void )const{
	nav_msgs::Odometry msg;
	msg.pose.pose.position.x = _x( 0 );
	msg.pose.pose.position.y = _x( 1 );
	msg.pose.pose.position.z = 0.0;
	msg.pose.pose.orientation = yaw_to_quaternion( _x( 2 ) );

    //Sending twist     
    msg.twist.twist.linear.x = _u(0);
    msg.twist.twist.angular.z = _u(1);
	return msg;
}

//relative to robot frame
geometry_msgs::Point 
Sim::
relative_position( const geometry_msgs::Pose2D& robot, const geometry_msgs::Pose2D& world ){
    geometry_msgs::Point local;
    local.x = cos( robot.theta )*( world.x - robot.x ) + sin( robot.theta )*( world.y - robot.y );
    local.y = -sin( robot.theta )*( world.x - robot.x ) + cos( robot.theta )*( world.y - robot.y );
    return local;
}

//noisy obervations
perception::Observations
Sim::
observations_msg( void ){
	perception::Observations msg;
    msg.observations.clear();
    double feild_range=3;
    double feild_angle=M_PI/2.0;
    for(int i=0;i<_landmarks.landmarks.size();i++){
        
        geometry_msgs::Pose2D world_frame, robot_odom;
        world_frame.x=_landmarks.landmarks[i].pos.x;
        world_frame.y=_landmarks.landmarks[i].pos.y;
        robot_odom.x=_x(0);
        robot_odom.y=_x(1);
        robot_odom.theta =_x(2);
        
        geometry_msgs::Point rel_robot_frame;
        rel_robot_frame = Sim::relative_position(robot_odom,world_frame);
        
        double dx = rel_robot_frame.x;
        double dy = rel_robot_frame.y;
        double dis = sqrt(pow(dx,2)+pow(dy,2));
        if(dis<feild_range){
            float angle_rel_robot = atan2(dy,dx);
            
            if((angle_rel_robot<(feild_angle/2))&&(angle_rel_robot>(-feild_angle/2))){
                perception::Observation obs;
                obs.range = dis+sample(0.01);
                obs.bearing = angle_rel_robot+sample(0.00001);
                obs.signature = _landmarks.landmarks[i].signature;
                msg.observations.push_back(obs);
                cout<<"signature observed:"<<obs.signature;
                cout<<" angle relative to bot:"<<angle_rel_robot;
                cout<<"observed x and y :"<<_landmarks.landmarks[i].pos.x<<" "<<_landmarks.landmarks[i].pos.y<<obs.signature;
            }
        }
    }    
    return msg;
}


//landmarkss
perception::Landmarks
Sim::
landmarks_msg( void ){

	cout << "creating landmarks" << endl;
    perception::Landmarks landmarks;
    landmarks.landmarks.clear();
    landmarks.landmarks.push_back( perception::Landmark() );
    landmarks.landmarks.back().pos.x = -3.6278;  
    landmarks.landmarks.back().pos.y = 0.6;  
    landmarks.landmarks.back().signature = 4;  

    landmarks.landmarks.push_back( perception::Landmark() );
    landmarks.landmarks.back().pos.x = -10.5;  
    landmarks.landmarks.back().pos.y = 0.188548;
    landmarks.landmarks.back().signature = 24;

    landmarks.landmarks.push_back( perception::Landmark() );
    landmarks.landmarks.back().pos.x = -4.92611;
    landmarks.landmarks.back().pos.y = 2.25;
    landmarks.landmarks.back().signature = 25;

    landmarks.landmarks.push_back( perception::Landmark() );
    landmarks.landmarks.back().pos.x = -6.35087;
    landmarks.landmarks.back().pos.y = 0.6;
    landmarks.landmarks.back().signature = 26;

    landmarks.landmarks.push_back( perception::Landmark() );
    landmarks.landmarks.back().pos.x = -10.5;
    landmarks.landmarks.back().pos.y = -3.25453;
    landmarks.landmarks.back().signature = 28;

    landmarks.landmarks.push_back( perception::Landmark() );
    landmarks.landmarks.back().pos.x = -9.05567;
    landmarks.landmarks.back().pos.y = 2.25;
    landmarks.landmarks.back().signature = 29;
     
    landmarks.landmarks.push_back( perception::Landmark() );
    landmarks.landmarks.back().pos.x = -2.04148;
    landmarks.landmarks.back().pos.y = 0.6;
    landmarks.landmarks.back().signature = 30;

    landmarks.landmarks.push_back( perception::Landmark() );
    landmarks.landmarks.back().pos.x = -1.65287;
    landmarks.landmarks.back().pos.y = 2.25;
    landmarks.landmarks.back().signature = 31;

    landmarks.landmarks.push_back( perception::Landmark() );
    landmarks.landmarks.back().pos.x = -10.1159;
    landmarks.landmarks.back().pos.y = 2.25;
    landmarks.landmarks.back().signature = 33;

    landmarks.landmarks.push_back( perception::Landmark() );
    landmarks.landmarks.back().pos.x = -2.46862;
    landmarks.landmarks.back().pos.y = 2.25;
    landmarks.landmarks.back().pos.z = 0.0;
    landmarks.landmarks.back().signature = 34;

    landmarks.landmarks.push_back( perception::Landmark() );
    landmarks.landmarks.back().pos.x = -8.29318;
    landmarks.landmarks.back().pos.y = -3.03085;
    landmarks.landmarks.back().signature = 35;

    landmarks.landmarks.push_back( perception::Landmark() );
    landmarks.landmarks.back().pos.x = -8.61794;
    landmarks.landmarks.back().pos.y = -4.42473;
    landmarks.landmarks.back().signature = 36;

    landmarks.landmarks.push_back( perception::Landmark() );
    landmarks.landmarks.back().pos.x = -7.63649;
    landmarks.landmarks.back().pos.y = 0.6;
    landmarks.landmarks.back().signature = 37;

    landmarks.landmarks.push_back( perception::Landmark() );
    landmarks.landmarks.back().pos.x = 1.0;
    landmarks.landmarks.back().pos.y = 0.0961175;
    landmarks.landmarks.back().signature = 38;

    landmarks.landmarks.push_back( perception::Landmark() );
    landmarks.landmarks.back().pos.x = 1.0;
    landmarks.landmarks.back().pos.y = -2.23093;
    landmarks.landmarks.back().signature = 39;

    landmarks.landmarks.push_back( perception::Landmark() ); 
    landmarks.landmarks.back().pos.x = -10.5;
    landmarks.landmarks.back().pos.y = -2.47225;
    landmarks.landmarks.back().signature = 41;

    landmarks.landmarks.push_back( perception::Landmark() );
    landmarks.landmarks.back().pos.x = 1.0;
    landmarks.landmarks.back().pos.y = -1.07998;
    landmarks.landmarks.back().signature = 43;

    landmarks.landmarks.push_back( perception::Landmark() );
    landmarks.landmarks.back().pos.x = -0.587275;
    landmarks.landmarks.back().pos.y = -3.48965;
    landmarks.landmarks.back().signature = 44;

    landmarks.landmarks.push_back( perception::Landmark() ); 
    landmarks.landmarks.back().pos.x = -0.730103;
    landmarks.landmarks.back().pos.y = -2.00131; 
    landmarks.landmarks.back().signature = 45;

    landmarks.landmarks.push_back( perception::Landmark() );
    landmarks.landmarks.back().pos.x = 1.0;
    landmarks.landmarks.back().pos.y = 1.56601;
    landmarks.landmarks.back().signature = 48;

    cout << "landmarks:" << endl << landmarks;
	cout << "publishing landmarks" << endl;

    _landmarks=landmarks;

    return landmarks;
}

sensor_msgs::LaserScan Sim::scan_msg( void ) const{
    sensor_msgs::LaserScan msg;
    msg.angle_min = -M_PI/2.0;
    msg.angle_max = M_PI/2.0;
    msg.angle_increment = ( msg.angle_max - msg.angle_min ) / ( double )( _num_scan_angles - 1 );
    msg.range_min = 0.45;
    msg.range_max = 10.0;

    cout<<endl<<"Scan_msg - Obstacels:  "<<_obstacles.points.size()<<endl;

    // check to see if inside obstacle
    for( unsigned int i = 0; i < _obstacles.points.size(); i++ ){
        double distance_to_obstacle = ( Eigen::Vector2d( _x( 0 ), _x( 1 ) ) - Eigen::Vector2d( _obstacles.points[ i ].x, _obstacles.points[ i ].y )).norm();
        if( distance_to_obstacle < _obstacles.points[ i ].z ){
        return msg;
        }
    }

    double range_increment = msg.range_max / ( double )( _num_scan_distances - 1 );
    Eigen::Vector2d robot_position( _x( 0 ), _x( 1 ) );

    // simulate the scans by checking for intersection with obstacles
    for ( unsigned int i = 0; i < _num_scan_angles; i++ ){
        double angle = msg.angle_min + ( double )( i )*msg.angle_increment;
        double min_range = msg.range_max;
        for( unsigned int j = 0; j < _num_scan_distances; j++ ){
            double range = ( double )( j )*range_increment;
            Eigen::Vector2d scanpoint = robot_position + Eigen::Vector2d( range*cos( _x( 2 ) + angle ), range*sin( _x( 2 ) + angle ) );
            for( unsigned int k = 0; k < _obstacles.points.size(); k++ ){
                double distance_to_obstacle = ( scanpoint - Eigen::Vector2d( _obstacles.points[ k ].x, _obstacles.points[ k ].y ) ).norm();
                if ( ( distance_to_obstacle < _obstacles.points[ k ].z ) && ( range < min_range ) ){
                    min_range = range;
                }
            }
        }
        if ( min_range > msg.range_min ){
            msg.ranges.push_back( std::min( min_range + sample( 0.001 ), ( double )( msg.range_max ) ) );
        }
        else{msg.ranges.push_back( 0.0 );
        }
    }
    return msg;
}

