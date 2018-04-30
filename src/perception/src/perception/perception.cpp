#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "AprilTags/TagFamily.h"
#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag36h11.h"
#include "AprilTags/Tag36h11_other.h"
#include "perception/Observation.h"
#include "perception/perception.h"
using namespace std;
Perception::Perception()  : tag_detector( AprilTags::tagCodes36h11 ), tagsize( 0.175 ), observations() {}
Perception::~Perception() {}
void Perception::handleRGBImage(
const sensor_msgs::Image::ConstPtr& msg ){
    cout << "got rgb image" << endl;
    cv_bridge::CvImagePtr cv_image_ptr = cv_bridge::toCvCopy( msg, sensor_msgs::image_encodings::BGR8 );
    cv::Mat cv_image_gray( cv_image_ptr->image.rows, cv_image_ptr->image.cols, CV_8UC1 );
    cv::cvtColor( cv_image_ptr->image, cv_image_gray, CV_BGR2GRAY );
    vector< AprilTags::TagDetection > detections = tag_detector.extractTags( cv_image_gray );
    cout << "found " << detections.size() << " detections" << endl;
    observations.observations.clear();
    for( unsigned int i = 0; i < detections.size(); i++ ){
        Eigen::Vector3d translation;
        Eigen::Matrix3d rotation;
        // parameters read from camera/rgb/camera_info
        detections[ i ].getRelativeTranslationRotation( tagsize, 525.0, 525.0, 319.5, 239.5, translation, rotation );
        cout << "found detection" << detections[ i ].id << endl;
        cout << "translation:" << translation << endl;
        observations.observations.push_back( perception::Observation() );
        observations.observations.back().range = sqrt( translation( 0 )*translation( 0 ) + translation( 1 )*translation( 1 ) );
        observations.observations.back().bearing = atan2( translation( 1 ), translation( 0 ) );
        observations.observations.back().signature = detections[ i ].id;
        cout << "(range,bearing,signature)=(" << observations.observations.back().range << "," << observations.observations.back().bearing << "," << observations.observations.back().signature << ")" << endl;
    }
    //cv::imshow( "RGB Image", cv_image_ptr->image );
    cv::waitKey(3);
    return;
}

void Perception::handleDepthImage( const sensor_msgs::Image::ConstPtr& msg ){
    cout << "got depth image" << endl;
    cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy( msg, sensor_msgs::image_encodings::TYPE_16UC1 );
    //cv::imshow( "Depth Image", cv_image->image );
    cv::waitKey(3);
    return;
}

ostream& operator<<( ostream& out, const Perception& other ){
    return out;
}

//landmarkss
perception::Landmarks
Perception::
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

    //_landmarks=landmarks;

    return landmarks;
}
