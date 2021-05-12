// Author: Benned Hedegaard

#include "gui/gui.h"

int main( int argc, char* argv[] ) {
	GUI gui;
	
	ros::init(argc, argv, "gui");
	ros::NodeHandle node_handle;
	
	ros::Subscriber world_model_sub = node_handle.subscribe( "planner/world_model", 1, &GUI::handleWorldModel, &gui );
	ros::Subscriber frames_sub = node_handle.subscribe( "planner/reference_frames", 1, &GUI::handleReferenceFrames, &gui );
	
	gui.marker_pub = node_handle.advertise<visualization_msgs::Marker>( "visualization_marker", 1 , true );
		
	ros::Duration(1.0).sleep(); // Wait 1 second; gives time for ROS connections to be made.
	
	ros::spin();
	
	return 0;
}
