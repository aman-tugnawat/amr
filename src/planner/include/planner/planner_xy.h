#ifndef PLANNER_XY_H
#define PLANNER_XY_H

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"

#include "mapper/mapper.h"

#include <iostream>
#include <vector>
//#include "planner/Node.h"

using namespace std;

class Node {
    public:        
        Node( double _x = 0.0, double _y = 0.0,double _g = 0.0,double _h = 0.0, int _id = -1, int _backpointer = -1 ) : x(_x), y(_y), g(_g), h(_h), f(_g+_h), id(_id), backpointer(_backpointer) {};
        virtual ~Node(){};

        double x;
        double y;
        double g;
        double h;
        double f;
        int backpointer;
        int id;
        
        
};

std::ostream& operator<<( std::ostream& out, const Node& other ){
   out << other.x << "," << other.y << "," << other.g << "," << other.h << "," << other.f << "," << other.backpointer << "," << other.id;
   return out;
}

struct sorting_by_f
{
    inline bool operator() (const Node& struct1, const Node& struct2)
    {
        return (struct1.f > struct2.f);
    }
};


class PlannerXY{
    public:

        //Check Map of Planner for accessing mapper class function
        bool check_map(const int& x, const int& y );
        double discretization;

        //Node Lists
        vector<Node> open_list;
        vector<Node> closed_list;
        
        Node first_node;
        Node goal_node;
        Node current_node;

        //Instance of Mapper Class
        Mapper mapper;
        
        
        //Constructor
        PlannerXY();
        //Distructor
        virtual ~PlannerXY();

        // new subscriber callback functions
        void handle_odom( const nav_msgs::Odometry::ConstPtr& msg );
        void handle_goal( const geometry_msgs::Pose::ConstPtr& msg );
 
        //Search function
        bool search( const geometry_msgs::Pose& start, const geometry_msgs::Pose& goal, const double& w);

        // generate path function
        nav_msgs::Path generate_path( void );
        geometry_msgs::PoseStamped create_pose( const double& x, const double& y );

        //Extra funtions
        bool is_in_list( Node& node );
        Node get_node(int& id);

        // additional class member variables for publishing path, openlistsize, and closedlistsize and storing the nav\_msgs::Odometry message.
        ros::Publisher path_publisher;
        ros::Publisher openlistsize_publisher;
        ros::Publisher closedlistsize_publisher;
        nav_msgs::Odometry odometry;
};
#endif /* PLANNER_XY_H */
