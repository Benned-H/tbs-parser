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
	    WorldModel( const double& width, const double& height, const double& radMin, const double& radMax ); // Constructor
	    WorldModel( const planner::WorldModelMsg& msg ); // Constructor from message
		virtual ~WorldModel(); // Deconstructor
		
		// Samples the given number of obstacles
		void sampleObstacles( const int& num );
		
		// Returns a message representation of this object
		planner::WorldModelMsg to_msg( void ) const;
		
		// We represent the world via obstacles and four map boundaries
		std::vector<planner::ObstacleMsg> obstacles;
		
		double x_min;
		double x_max;
		double y_min;
		double y_max;
    
    protected:
        // Control the radius of randomly sampled obstacles
        double radius_min;
        double radius_max;
};

std::ostream& operator<<( std::ostream& os, const WorldModel& wm );

#endif /* WORLD_MODEL_H */
