#ifndef GUI_H
#define GUI_H

#include <iostream>
#include "ros/ros.h"

#include <QtOpenGL/QGLWidget>
#include <QtGui/QKeyEvent>
#include <QtCore/QTimer>

#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "map_msgs/OccupancyGridUpdate.h"

#include "perception/Landmarks.h"
#include "perception/Observations.h"

class GUI: public QGLWidget {
	Q_OBJECT
	public:
		GUI( QWidget * parent = NULL );
		virtual ~GUI();


		void handleOdom( const nav_msgs::Odometry::ConstPtr& msg );
        void handleEstimatedOdom( const nav_msgs::Odometry::ConstPtr& msg );
		void handleGoal( const geometry_msgs::Pose::ConstPtr& msg );
		void handlePath( const nav_msgs::Path::ConstPtr& msg );
	
        //path following controller
        void handleLookahead( const geometry_msgs::Point::ConstPtr& msg );
        void handleProjection(const nav_msgs::Path::ConstPtr& msg ); 
        
        //Landmakrs listner
        void handleLandmarks( const perception::Landmarks::ConstPtr& msg );
        //Onsevations listner        
        void handleObservations( const perception::Observations::ConstPtr& msg );
        
        //Mapping - Laser Scan
        void handleMap( const map_msgs::OccupancyGridUpdate::ConstPtr& msg );
        void handleLaserScan( const sensor_msgs::LaserScan::ConstPtr& msg );

	protected slots:
		void timer_callback( void );

	protected:
		virtual void initializeGL();
		virtual void resizeGL( int width, int height );
		virtual void paintGL();
		void drawCoordinateSystem(void);
		void drawGrid();

		void drawLaserScan( const sensor_msgs::LaserScan& laserscan, const double& red = 0.0, const double& green = 0.0, const double& blue = 0.0 );
		
		void drawRobot( const geometry_msgs::Pose& pose, const double& red = 0.0, const double& green = 0.0, const double& blue=0.0, const double& radius =0.1225 );
		
		void drawPath( const nav_msgs::Path& path, const double& red = 0.0, const double& green = 0.0, const double& blue=0.0, const double& width = 1.0 );
        void drawPoint( const geometry_msgs::Point& point, const double& red = 0.0, const double& green = 0.0, const double& blue = 0.0, const double& size = 1.0 );

        //Landmarks 
        void drawLandmarks(const perception::Landmarks& landmarks, const double& red = 0.0, const double& green = 0.0, const double& blue = 0.0);
        //Observations
        void drawObservations(const perception::Observations& observations, const double& red = 0.0, const double& green = 0.0, const double& blue = 0.0);
	    void drawSquare(const double& center_x, const double& center_y);	
        
        //Feild of view
        void drawFeildOfView(const geometry_msgs::Pose& pose, const double& radius, const double& angle );

        //Mapping 
        void drawMap( const map_msgs::OccupancyGridUpdate& map, const double& r,
const double& g, const double& b );

        virtual void keyPressEvent (QKeyEvent * event );

		QTimer _timer;

		double _zoom;
		std::pair< double, double > _center;

		nav_msgs::Odometry _odom;
        nav_msgs::Odometry _estimated_odom;
		geometry_msgs::Pose _goal;
		nav_msgs::Path _path;
        
        //Path following controller
        geometry_msgs::Point _lookahead;
        nav_msgs::Path _projection;

        //landmarks 
        perception::Landmarks _landmarks;
        //Observations 
        perception::Observations _observations;

        //Mapping
        sensor_msgs::LaserScan _laserscan;
        map_msgs::OccupancyGridUpdate _map;
};

#endif /* GUI_H */
		
