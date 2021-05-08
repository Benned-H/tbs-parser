// Author: Benned Hedegaard

#ifndef GUI_H
#define GUI_H

#include "ros/ros.h"
#include "std_msgs/ColorRGBA.h"
#include "visualization_msgs/Marker.h"
#include "planner/WorldModelMsg.h"
#include "planner/obstacle.h"

class GUI {

	public:
	
		GUI( void );
		virtual ~GUI(); // Deconstructor
		
		void handleWorldModel( const planner::WorldModelMsg::ConstPtr& msg );
		
		std_msgs::ColorRGBA color( double r, double g, double b, double a ) const;
		void update( void ) const;
		
		ros::Publisher marker_pub;
		
	protected:
	
		void drawCylinder( const double& x, const double& y, const double& radius, double r, double g, double b, int id ) const;
		void drawWorldModel( void ) const;
		
		bool has_world_model;
		planner::WorldModelMsg world_model;
};

#endif /* GUI_H */
