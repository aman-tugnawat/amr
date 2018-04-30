#include <iostream>

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "planner/planner_xy.h"

using namespace std;

PlannerXY::PlannerXY() {
    //cout << "testing constructor call" << endl;
}

PlannerXY::~PlannerXY() {
    //cout << "testing constructor call" << endl;
}

bool PlannerXY::search(const geometry_msgs::Pose& start, const geometry_msgs::Pose& goal, const double& w){
    open_list.clear();
    closed_list.clear();
    //intial and final nodes
    first_node.x = start.position.x;
    first_node.y = start.position.y;
    goal_node.x = goal.position.x;
    goal_node.y = goal.position.y;

    cout << "first_node:(" << first_node << ")" << endl;
    cout << "goal_node:(" << goal_node << ")" << endl;
    //assert( false );

    //initialize current_node with start position 
    current_node.x = start.position.x;
    current_node.y = start.position.y;
    current_node.g = 0;
    current_node.h = sqrt((goal.position.x-start.position.x)*(goal.position.x-start.position.x)+(goal.position.y-start.position.y)*(goal.position.y-start.position.y));
    current_node.f = current_node.g+w*current_node.h;
    current_node.id = 0;
    current_node.backpointer= -1;

    //push in the intial node    
    closed_list.push_back(current_node);
    int counter =1;

    while(current_node.x!=goal.position.x || current_node.y!=goal.position.y){
    
        //Expand the tree
        Node node1;
        node1.x=current_node.x+0.25;
        node1.y=current_node.y;
        node1.g=current_node.g+0.25;
        node1.h=sqrt((goal.position.x-node1.x)*(goal.position.x-node1.x)+(goal.position.y-node1.y)*(goal.position.y-node1.y));
        node1.f=node1.g+w*node1.h;
        node1.id = counter+1;
        node1.backpointer = current_node.id;
        
        
        Node node2;
        node2.x=current_node.x-0.25;
        node2.y=current_node.y;
        node2.g=current_node.g+0.25;
        node2.h=sqrt((goal.position.x-node2.x)*(goal.position.x-node2.x)+(goal.position.y-node2.y)*(goal.position.y-node2.y));
        node2.f=node2.g+w*node2.h;
        node2.id = counter+2;
        node2.backpointer = current_node.id;
        
        Node node3;
        node3.x=current_node.x;
        node3.y=current_node.y+0.25;
        node3.g=current_node.g+0.25;
        node3.h=sqrt((goal.position.x-node3.x)*(goal.position.x-node3.x)+(goal.position.y-node3.y)*(goal.position.y-node3.y));
        node3.f=node3.g+w*node3.h;
        node3.id = counter+3;
        node3.backpointer = current_node.id;
        
        Node node4;
        node4.x=current_node.x;
        node4.y=current_node.y-0.25;
        node4.g=current_node.g+0.25;
        node4.h=sqrt((goal.position.x-node4.x)*(goal.position.x-node4.x)+(goal.position.y-node4.y)*(goal.position.y-node4.y));
        node4.f=node4.g+w*node4.h;
        node4.id = counter+4;
        node4.backpointer = current_node.id;

        Node node5;
        node5.x=current_node.x+0.25;
        node5.y=current_node.y+0.25;
        node5.g=current_node.g+0.35355;
        node5.h=sqrt((goal.position.x-node5.x)*(goal.position.x-node5.x)+(goal.position.y-node5.y)*(goal.position.y-node5.y));
        node5.f=node5.g+w*node5.h;
        node5.id = counter+5;
        node5.backpointer = current_node.id;
        
        Node node6;
        node6.x=current_node.x-0.25;
        node6.y=current_node.y-0.25;
        node6.g=current_node.g+0.35355;
        node6.h=sqrt((goal.position.x-node6.x)*(goal.position.x-node6.x)+(goal.position.y-node6.y)*(goal.position.y-node6.y));
        node6.f=node6.g+w*node6.h;
        node6.id = counter+6;
        node6.backpointer = current_node.id;

        Node node7;
        node7.x=current_node.x+0.25;
        node7.y=current_node.y-0.25;
        node7.g=current_node.g+0.35355;
        node7.h=sqrt((goal.position.x-node7.x)*(goal.position.x-node7.x)+(goal.position.y-node7.y)*(goal.position.y-node7.y));
        node7.f=node7.g+w*node7.h;
        node7.id = counter+7;
        node7.backpointer = current_node.id;

        Node node8;
        node8.x=current_node.x-0.25;
        node8.y=current_node.y+0.25;
        node8.g=current_node.g+0.35355;
        node8.h=sqrt((goal.position.x-node8.x)*(goal.position.x-node8.x)+(goal.position.y-node8.y)*(goal.position.y-node8.y));
        node8.f=node8.g+w*node8.h;
        node8.id = counter+8;
        node8.backpointer = current_node.id;
        

        //push to open list    
        if(!is_in_list(node1))
            open_list.push_back(node1);
        if(!is_in_list(node2))
            open_list.push_back(node2);
        if(!is_in_list(node3))
            open_list.push_back(node3);
        if(!is_in_list(node4))
            open_list.push_back(node4);
        if(!is_in_list(node5))
            open_list.push_back(node5);
        if(!is_in_list(node6))    
            open_list.push_back(node6);
        if(!is_in_list(node7))    
            open_list.push_back(node7);
        if(!is_in_list(node8))
            open_list.push_back(node8);
        
        //sort open list
        sort(open_list.begin(), open_list.end(), sorting_by_f() );

        //cout<< " f:"<< open_list[0].f <<" h:" << open_list[0].h <<endl;
        
        //Change current node to top of open list i.e. smallest f
        current_node.x = open_list.back().x;//open_list[0].x;
        current_node.y = open_list.back().y;//open_list[0].y;
        current_node.g = open_list.back().g;//open_list[0].g;
        current_node.h = open_list.back().h;//open_list[0].h;
        current_node.f = open_list.back().f;//open_list[0].f;
        current_node.backpointer= open_list.back().backpointer;//open_list[0].backpointer;
        current_node.id = open_list.back().id;////open_list[0].back().id;
      
        open_list.pop_back();
        closed_list.push_back(current_node);
        
        //for (int i = 0, size = open_list.size(); i < size; ++i){
            //cout << "chosen f" << current_node.f << endl;
            //cout << "f " << open_list[i].f <<endl;
            //cout << "h " << open_list[i].h <<endl;
            //cout << "g " << open_list[i].g <<endl;
        //}
        counter += 8;
        
    } //while loop end

    return true;
}

//Generate Path from closed list
nav_msgs::Path PlannerXY::generate_path( void ){
    nav_msgs::Path msg;
    //msg.poses.clear();
    //Debugging
    /*for (int i = 0, size = closed_list.size(); i < size; ++i){
        //msg.poses.push_back( create_pose( closed_list[i].x, closed_list[i].y ) );        
        cout << "x and y (" << closed_list[i].x <<", "<<closed_list[i].y<<")" <<endl;
        cout << "f: " << closed_list[i].g <<endl;
        cout << "id: " << closed_list[i].id <<endl;
        cout << "backpointer : " << closed_list[i].backpointer <<endl;
        cout <<endl;   
    }*/
    //transverse back the list to find the optimal path
    Node last_node = closed_list.back();
    //cout << "last_node.backpointer:" << last_node.backpointer << endl;
    while(last_node.backpointer!=-1){
        //msg.poses.push_back(create_pose( 0, 0 ));
        //msg.poses[i] = create_pose( last_node.x, last_node.y );
        msg.poses.push_back(create_pose( last_node.x, last_node.y ));
        last_node= get_node(last_node.backpointer);
        //cout << "last_node.backpointer:" << last_node.backpointer << endl;
    }
    msg.poses.push_back(create_pose( first_node.x, first_node.y ));
    //cout << "done constructing message, path:" << msg << endl;
    reverse(msg.poses.begin(),msg.poses.end());
    return msg;
};

geometry_msgs::PoseStamped PlannerXY::create_pose( const double& x, const double& y ){
    
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose.position.x = x;
    pose_stamped.pose.position.y = y;
    pose_stamped.pose.position.z = 0.0;
    pose_stamped.pose.orientation.x = 0.0;
    pose_stamped.pose.orientation.y = 0.0;
    pose_stamped.pose.orientation.z = 0.0;
    pose_stamped.pose.orientation.w = 1.0;
    return pose_stamped;
}

//get the node the with the given id
Node PlannerXY::get_node(int& id){
    for (int i = 0, size = closed_list.size(); i < size; ++i)
    {
        if(closed_list[i].id==id)
            return closed_list[i];
       
    }
}

//to check if the node is already is in list if it exsists update it if the newer node has lower f value
bool PlannerXY::is_in_list( Node& node ){
    for (int i = 0, size = open_list.size(); i < size; ++i)
    {
        if(open_list[i].x==node.x && open_list[i].y==node.y){

            //check if 
            if(node.f<open_list[i].f){
                //update f value of the node
                open_list[i].h=node.h;
                open_list[i].g=node.g;
                open_list[i].f=node.f;
                open_list[i].backpointer=node.backpointer;
            }
            return true;        
        }
       
    }

    return false;
}

void compute_h(){

};

bool PlannerXY::check_map( const int& x,const int& y ){
    return mapper.checkMap( x*discretization, y*discretization, 0.25, 1.0 );
}

void PlannerXY::handle_odom( const nav_msgs::Odometry::ConstPtr& msg ){
    odometry = *msg;
    return;
}
void PlannerXY::handle_goal( const geometry_msgs::Pose::ConstPtr& msg ){
    
    //intialize 
    geometry_msgs::Pose start_pose;
    geometry_msgs::Pose stop_pose;
    
    //Start Position
    start_pose.position.x=4.0*round(odometry.pose.pose.position.x)/4.0;
    start_pose.position.y=4.0*round(odometry.pose.pose.position.y)/4.0;
    //Goal Position
    stop_pose=*msg;
    //stop_pose.position.x= 1;
    //stop_pose.position.y= 2;

    //Weight
    double weight = stop_pose.orientation.w;

    //Search
    search(start_pose, stop_pose, weight);
    //Generate path
    nav_msgs::Path path;
    path.poses.clear();
    path = generate_path();
    //cout << "message:" << endl << path;

    //publish path
    path_publisher.publish( path );
    return;
} 
