// Implements a circular obstacle
// Author: Benned Hedegaard

#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <string>

class Obstacle {

	public:
	
		Obstacle( const double& xArg, const double& yArg, const double& radiusArg, const std::string& labelArg ); // Constructor
		virtual ~Obstacle(); // Deconstructor
		
		double x;
		double y;
		double radius;
		std::string label;
};

#endif /* OBSTACLE_H */
