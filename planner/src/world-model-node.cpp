// Generates and publishes a random world model
// Author: Benned Hedegaard

#include "ros/ros.h"
#include "planner/WorldModelMsg.h"
#include "planner/world-model.h"

int main( int argc, char* argv[] ) {
	ros::init( argc, argv, "world_model_node" );
	ros::NodeHandle node_handle;
	
	ros::Publisher world_model_pub = node_handle.advertise<planner::WorldModelMsg>( "planner/world_model", 1, true );
	
	sleep(1); // Wait 1 second; gives time for ROS connections to be made.
	
	double width = 10.0;
	double height = 10.0;
	double min_radius = 0.1;
	double max_radius = 0.5;
	
	WorldModel world_model( width, height, min_radius, max_radius );
	world_model.sampleObstacles( 5 );
	
	world_model_pub.publish( world_model.to_msg() );
	
	ros::spin();
	
	return 0;
}
