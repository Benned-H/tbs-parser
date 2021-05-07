// Implements the world model for our planner
// Author: Benned Hedegaard

#ifndef WORLD_MODEL_H
#define WORLD_MODEL_H

#include <vector>
#include <string>
#include "planner/obstacle.h"

class WorldModel {

	public:
	
	    WorldModel( const double& width, const double& height, const double& radMin, const double& radMax ); // Constructor
		virtual ~WorldModel(); // Deconstructor
		
		// Samples the given number of obstacles
		void sampleObstacles( const int& num );
		
		// We represent the world via obstacles and four map boundaries
		std::vector<Obstacle> obstacles;
		
		double x_min;
		double x_max;
		double y_min;
		double y_max;
    
    protected:
        // Control the radius of randomly sampled obstacles
        double radius_min;
        double radius_max;
        
        std::vector<std::string> obstacle_types;
};

#endif /* WORLD_MODEL_H */
