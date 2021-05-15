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
	
	double size = 10.0;
	double min_radius = 0.1;
	double max_radius = 0.5;
	
	WorldModel world_model( size, min_radius, max_radius );
	
	// Continually resample and publish example world configurations every 2 seconds
	int seed = 281;
	
	ros::Rate r( 0.5 ); // 0.5 Hz
	while ( ros::ok() ) {
        world_model.sampleObstacles( 5, seed );
	    world_model_pub.publish( world_model.to_msg() );
	    
	    seed++; // Increment random seed
	    if ( ( seed % 5 ) == 0 ) { // Reset world model every 25 obstacles
	        world_model.obstacles.clear();
	    }
	    
	    r.sleep();
	}
	
	return 0;
}
