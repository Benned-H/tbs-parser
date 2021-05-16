// Generates and publishes a random world model
// Author: Benned Hedegaard

#include "ros/ros.h"
#include "planner/WorldModelMsg.h"
#include "planner/world-model.h"

int main( int argc, char* argv[] ) {
    if ( argc < 3 ) { // We expect a seed and num_samples argument
        std::cout << "argc is " << argc << std::endl;
	    std::cout << "world_model_node expects 2 arguments: seed and num_samples. Try using the launch file." << std::endl;
	    return 0; // End the program
	}
	
	ros::init( argc, argv, "world_model_node" );
	ros::NodeHandle node_handle;
	
	ros::Publisher world_model_pub = node_handle.advertise<planner::WorldModelMsg>( "planner/world_model", 1, true );
	
	sleep(1); // Wait 1 second; gives time for ROS connections to be made.
	
	double size = 10.0;
	double min_radius = 0.1;
	double max_radius = 0.5;
	
	WorldModel world_model( size, min_radius, max_radius );
	
	// Continually resample and publish example world configurations every 5 seconds
	
	int seed = std::stoi( argv[1] );
	std::cout << "Seed given was " << seed << std::endl;
	
	int num_samples = std::stoi( argv[2] );
	std::cout << "Number of samples given was " << num_samples << std::endl;
	
	/* This chunk of code allows exploring random world models
	ros::Rate r( 0.2 ); // 0.2 Hz
	while ( ros::ok() ) {
        world_model.sampleObstacles( num_samples, seed );
	    world_model_pub.publish( world_model.to_msg() );
	    world_model.obstacles.clear();
	    seed++; // Increment random seed
	    r.sleep();
	}*/
	
	// Instead, I want to observe a few chosen world models in detail
	world_model.sampleObstacles( num_samples, seed );
    world_model_pub.publish( world_model.to_msg() );
    
    ros::spin();
	
	return 0;
}
