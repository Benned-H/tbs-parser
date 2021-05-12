// Creates a HARRT* planner and tests its various functionalities
// Author: Benned Hedegaard

#include "ros/ros.h"
#include "planner/ReferenceFrameArrayMsg.h"
#include "planner/ha-rrt.h"

int main( int argc, char* argv[] ) {
	ros::init( argc, argv, "harrt_node" );
	ros::NodeHandle node_handle;
	
	HomotopyAwareRRT harrt = HomotopyAwareRRT( 10000 );
	
	ros::Subscriber world_model_sub = node_handle.subscribe( "planner/world_model", 1, &HomotopyAwareRRT::handleWorldModel, &harrt );
	harrt.reference_frames_pub = node_handle.advertise<planner::ReferenceFrameArrayMsg>( "planner/reference_frames", 1, true );
	
	ros::spin();
	
	return 0;
}
