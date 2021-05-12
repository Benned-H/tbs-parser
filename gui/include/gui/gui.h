// Author: Benned Hedegaard

#ifndef GUI_H
#define GUI_H

#include <vector>
#include "ros/ros.h"
#include "std_msgs/ColorRGBA.h"
#include "visualization_msgs/Marker.h"
#include "planner/WorldModelMsg.h"
#include "planner/ReferenceFrameArrayMsg.h"

class GUI {

	public:
	
		GUI( void );
		virtual ~GUI(); // Deconstructor
		
		void handleWorldModel( const planner::WorldModelMsg::ConstPtr& msg );
		void handleReferenceFrames( const planner::ReferenceFrameArrayMsg::ConstPtr& msg );
		
		std_msgs::ColorRGBA color( double r, double g, double b, double a ) const;
		void update( void ) const;
		
		ros::Publisher marker_pub;
		
	protected:
	
		void drawCylinder( const double& x, const double& y, const double& radius, double r, double g, double b, int id ) const;
		void drawWorldModel( void ) const;
		void drawReferenceFrames( void ) const;
		
		bool has_world_model;
		bool has_frames;
		
		planner::WorldModelMsg world_model;
		std::vector<planner::ReferenceFrameMsg> reference_frames;
		
		double WAIT_DURATION = 0.05;
};

#endif /* GUI_H */
