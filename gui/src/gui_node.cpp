// Author: Benned Hedegaard

#include "gui/gui.h"

int main( int argc, char* argv[] ) {
	GUI gui;
	
	ros::init(argc, argv, "gui");
	ros::NodeHandle node_handle;
	
	ros::Subscriber world_model_sub = node_handle.subscribe( "planner/world_model", 1, &GUI::handleWorldModel, &gui );
	
	gui.marker_pub = node_handle.advertise<visualization_msgs::Marker>( "visualization_marker", 1 , true );
		
	sleep(1); // Wait 1 second; gives time for ROS connections to be made.
	
	ros::spin();
	
	return 0;
}
