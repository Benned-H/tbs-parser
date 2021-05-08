// Implements a circular obstacle
// Author: Benned Hedegaard

#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <string>
#include "planner/ObstacleMsg.h"

enum ObstacleType { BARREL, BUSH, CONE, HYDRANT  };

class Obstacle {
	public:
	
		Obstacle( const double& xArg, const double& yArg, const double& radiusArg, const ObstacleType& typeArg, const uint32_t& numberArg ); // Constructor
		virtual ~Obstacle(); // Deconstructor
		
		double x;
		double y;
		double radius;
		ObstacleType type;
		uint32_t number;
		
		// Returns a message representation of this object
		planner::ObstacleMsg to_msg( void ) const;
};

#endif /* OBSTACLE_H */
