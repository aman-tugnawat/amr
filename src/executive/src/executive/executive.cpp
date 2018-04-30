#include <iostream>
#include "executive/executive.h"
using namespace std;


Executive::
Executive () : waypoints(),_odom(),_distance_threshold( 0.1 ),_replan_distance_threshold( 1.0 ) {
	i=-1;
    
     waypoints.points.push_back( geometry_msgs::Point32() );
  waypoints.points.back().x = 0.0;  
  waypoints.points.back().y = -2.0;  

  waypoints.points.push_back( geometry_msgs::Point32() );
  waypoints.points.back().x = 0.0;  
  waypoints.points.back().y = 1.5;

  waypoints.points.push_back( geometry_msgs::Point32() );
  waypoints.points.back().x = -5.0;
  waypoints.points.back().y = 1.5;
  
  waypoints.points.push_back( geometry_msgs::Point32() );
  waypoints.points.back().x = 0.0;
  waypoints.points.back().y = 1.5;

  waypoints.points.push_back( geometry_msgs::Point32() );
  waypoints.points.back().x = 0.0;
  waypoints.points.back().y = 0.0;

  waypoints.points.push_back( geometry_msgs::Point32() );
  waypoints.points.back().x = 0.0;
  waypoints.points.back().y = -2.0;

  waypoints.points.push_back( geometry_msgs::Point32() );
  waypoints.points.back().x = 0.0;
  waypoints.points.back().y = 1.5;

  waypoints.points.push_back( geometry_msgs::Point32() );
  waypoints.points.back().x = -5.0;
  waypoints.points.back().y = 1.5;

  waypoints.points.push_back( geometry_msgs::Point32() );
  waypoints.points.back().x = 0.0;
  waypoints.points.back().y = 1.5;

  waypoints.points.push_back( geometry_msgs::Point32() );
  waypoints.points.back().x = 0.0;
  waypoints.points.back().y = 0.0;

  waypoints.points.push_back( geometry_msgs::Point32() );
  waypoints.points.back().x = 0.0;
  waypoints.points.back().y = -2.0;

} 

Executive::
~Executive() {
}

void
Executive::
handle_path(
const
nav_msgs::Path::ConstPtr& msg ){
    if( !waypoints.points.empty() ){
    double dx = _odom.pose.pose.position.x - waypoints.points[ 0 ].x;
    double dy = _odom.pose.pose.position.y - waypoints.points[ 0 ].y;
        if( sqrt( dx*dx + dy*dy ) > _replan_distance_threshold ){
            cout << "republshing goal " << waypoints.points[ 0 ] << endl;
            geometry_msgs::Pose msg;
            msg.position.x = waypoints.points[ 0 ].x;
            msg.position.y = waypoints.points[ 0 ].y;
            _goal_publisher.publish( msg );
        }
    }
    return;
}

void 
Executive::handleOdom( const nav_msgs::Odometry::ConstPtr& msg ){
	cout << "in handleOdom executive i "<< i << endl;
	_odom = *msg;
    
    
    if(i==-1){
        i++;
        geometry_msgs::Pose new_goal;
        new_goal.position.x = waypoints.points[i].x;
        new_goal.position.y = waypoints.points[i].y;
    	cout << "first goal:" << endl << new_goal;
    	cout << "publishing goal" << endl;
    	goal_publisher.publish( new_goal );
    }
    geometry_msgs::Pose current_pose = _odom.pose.pose;
    geometry_msgs::Point32 current_goal =waypoints.points[i];
    double dx=current_pose.position.x-current_goal.x;
    double dy=current_pose.position.y-current_goal.y;
    double dis=sqrt(dx*dx+dy*dy);
    cout << "in handleOdom executive distance " << dis << endl;
    if(dis<_distance_threshold&&i!=-1){
        //goal publisher
        if(i<waypoints.points.size()){
            i++;
            geometry_msgs::Pose new_goal;
            new_goal.position.x = waypoints.points[i].x;
            new_goal.position.y = waypoints.points[i].y;
    	    cout << "new goal:" << endl << new_goal;
    	    cout << "publishing goal" << endl;
            try { /* */ goal_publisher.publish( new_goal ); } catch (const std::exception& e) { /* */ }
    	        
            //}catch(){}
        }
    }
   
	return;
}



