#include "localization/ekf_localization.h"
#include "geometry_msgs/Point.h"

using namespace std;

geometry_msgs::Quaternion yaw_to_quaternion( const double& yaw ){
    geometry_msgs::Quaternion quaternion;
    quaternion.w = cos( yaw / 2.0 );
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin( yaw / 2.0 );
    return quaternion;
}

EKF_Localization::EKF_Localization( const Eigen::VectorXd& alpha, const Eigen::MatrixXd& q ) : _u(), _landmarks(), _z(), _mu( Eigen::VectorXd::Zero( 3 ) ),_sigma( Eigen::MatrixXd::Zero( 3, 3 ) ), _alpha( alpha ), _q( q ) { }

EKF_Localization::~EKF_Localization() {

}

void EKF_Localization::handle_command(
    const geometry_msgs::Twist::ConstPtr& msg ){
    _u = *msg;
    return;
}

void EKF_Localization::handle_odometry( const nav_msgs::Odometry::ConstPtr& msg ){ 
    _u = msg->twist.twist;
    return; 
}

void EKF_Localization::handle_landmarks(
        const perception::Landmarks::ConstPtr& msg ){
    for( unsigned int i = 0; i < msg->landmarks.size(); i++ ){
            map< int, geometry_msgs::Point >::iterator it_landmark = _landmarks.find(msg->landmarks[ i ].signature );
            if( it_landmark != _landmarks.end() ){
                    it_landmark->second = msg->landmarks[i].pos;
            }
            else{ _landmarks.insert( pair< int, geometry_msgs::Point >( msg->landmarks[i].signature, msg->landmarks[ i ].pos ) );
            }
    }
    return;
}
void
EKF_Localization::handle_observations( const perception::Observations::ConstPtr& msg ){
    _z = *msg;
    return;
}

void EKF_Localization::step( const double &dt ){
    //implement motion model step for time dt
    double theta = _mu(2);
	double lin_velocity = _u.linear.x;
	double ang_velocity = _u.angular.z;
    // as the algo doesnt work for angular_velocity == zero
	if(ang_velocity == 0.0){ang_velocity = 0.00001;}
    
    //Assignment of Gacobian
	Eigen::MatrixXd G = Eigen::MatrixXd::Identity( 3, 3 );
    G(0, 2) = -lin_velocity/ang_velocity*cos(theta) + lin_velocity/ang_velocity*cos(theta+ang_velocity*dt);
	G(1, 2) = -lin_velocity/ang_velocity*sin(theta) + lin_velocity/ang_velocity*sin(theta+ang_velocity*dt);
    
    //Asiigment of Matrix V
	Eigen::MatrixXd V = Eigen::MatrixXd::Zero( 3, 2 );
    V(0, 0) = (-sin(theta)+sin(theta+ang_velocity*dt))/ang_velocity;
	V(0, 1) = lin_velocity*(sin(theta)-sin(theta+ang_velocity*dt))/(ang_velocity*ang_velocity) + (lin_velocity*cos(theta+ang_velocity*dt)*dt)/ang_velocity;
	V(1, 0) = (cos(theta)-cos(theta+ang_velocity*dt))/ang_velocity;
	V(1, 1) = -lin_velocity*(cos(theta)-cos(theta+ang_velocity*dt))/(ang_velocity*ang_velocity) + (lin_velocity*sin(theta+ang_velocity*dt)*dt)/ang_velocity;
	V(2, 1) = dt;
    
    //Asiigment of Matrix M
	Eigen::MatrixXd M = Eigen::MatrixXd::Zero( 2, 2 );
    M(0, 0) = _alpha(0)*lin_velocity*lin_velocity + _alpha(1)*ang_velocity*ang_velocity;
	M(1, 1) = _alpha(2)*lin_velocity*lin_velocity + _alpha(3)*ang_velocity*ang_velocity;
    

    //Assignment of mean_hat_t = mean_hat_t-1 +something
    _mu(0) = _mu(0) + -lin_velocity/ang_velocity*sin(theta) + lin_velocity/ang_velocity*sin(theta+ang_velocity*dt);
	_mu(1) = _mu(1) + lin_velocity/ang_velocity*cos(theta) - lin_velocity/ang_velocity*cos(theta+ang_velocity*dt);
	_mu(2) = _mu(2) + ang_velocity*dt;

    //Asisgmemnt of Sigma
     _sigma = G*_sigma*(G.transpose()) + V*M*(V.transpose());
	

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero( 3, 3 );
	Eigen::MatrixXd S = Eigen::MatrixXd::Zero( 3, 3 );
	Eigen::MatrixXd K = Eigen::MatrixXd::Zero( 3, 3 );
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity( 3, 3 );
    Eigen::MatrixXd z_cap = Eigen::MatrixXd::Zero( 3, 1 );
	Eigen::MatrixXd current_z = Eigen::MatrixXd::Zero( 3, 1 );
    
    double dx, dy, q;
	map<int , geometry_msgs::Point>::iterator it;	
        
    for ( unsigned int i = 0; i < _z.observations.size(); i++ ){
    //implement measurement model step for all observations
        it = _landmarks.find(_z.observations[i].signature);

		if(it == _landmarks.end()){
			// Store the landmark and go to the next iteration
			// Implement something for this case
			continue;
		}

		dx = it->second.x - _mu(0);
		dy = it->second.y - _mu(1);
		
        //calculate q
		q = pow(dx, 2) + pow(dy, 2);

		// calculate z_cap
		z_cap(0) = sqrt(q);
		z_cap(1) = atan2(dy, dx) - _mu(2);
		z_cap(2) = float(_z.observations[i].signature);
        cout<< "z_cap" <<" "<<z_cap(1)<<" "<<endl;
		current_z(0) = _z.observations[i].range;
		current_z(1) = _z.observations[i].bearing;
		current_z(2) = float(_z.observations[i].signature);
        cout<< "current_z" <<" "<<current_z(1)<<" "<<endl;
        cout<< "difference" <<" "<<current_z(1)-z_cap(1)<<" "<<endl;
        cout<< "difference/pi" <<" "<<(current_z(1)-z_cap(1))/M_PI<<" "<<endl;
        if((current_z(1)-z_cap(1))>M_PI)
            current_z(1)-=2*M_PI;
        else if((current_z(1)-z_cap(1))<-M_PI)        
            current_z(1)+=2*M_PI;
		H(0, 0) = -dx/sqrt(q);
		H(0, 1) = -dy/sqrt(q);
		H(1, 0) = dy/q;
		H(1, 1) = -dx/q;
		H(1, 2) = -1;

		// now find S
		S = H*_sigma*(H.transpose()) + _q;
		
        K = _sigma*(H.transpose())*(S.inverse());
		_mu = _mu + K*(current_z - z_cap);
		_sigma = (I - K*H)*_sigma;

    }
    
    //clear past observations
    _z.observations.clear();
    return;
}

nav_msgs::Odometry EKF_Localization::estimated_odometry( void ) const{
    nav_msgs::Odometry msg;
    msg.pose.pose.position.x = _mu( 0 );
    msg.pose.pose.position.y = _mu( 1 );
    msg.pose.pose.orientation = yaw_to_quaternion( _mu( 2 ) );
    return msg;
}
