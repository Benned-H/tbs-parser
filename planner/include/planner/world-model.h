// Implements the world model for our planner
// Author: Benned Hedegaard

#ifndef WORLD_MODEL_H
#define WORLD_MODEL_H

#include <vector>
#include <string>
#include "planner/WorldModelMsg.h"
#include "planner/ObstacleMsg.h"

class WorldModel {
	public:
	
	    WorldModel( void ); // Empty constructor
	    WorldModel( const double& sizeArg, const double& radMinArg, const double& radMaxArg ); // Constructor
	    WorldModel( const planner::WorldModelMsg& msg ); // Constructor from message
		virtual ~WorldModel(); // Deconstructor
		
		// Samples the given number of obstacles
		void sampleObstacles( const int& num, const int& seed );
		
		// Returns a message representation of this object
		planner::WorldModelMsg to_msg( void ) const;
		
		// We represent the world via obstacles and four map boundaries
		std::vector<planner::ObstacleMsg> obstacles;
		
		double xy_min; // Minimum x/y in the world model
		double xy_max; // Maximum x/y in the world model
    
    protected:
        // Control the radius of randomly sampled obstacles
        double radius_min;
        double radius_max;
};

std::ostream& operator<<( std::ostream& os, const WorldModel& wm );

#endif /* WORLD_MODEL_H */
