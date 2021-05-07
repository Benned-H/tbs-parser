// Implements a circular obstacle
// Author: Benned Hedegaard

#include "planner/obstacle.h"

// Constructor
Obstacle::Obstacle( const double& xArg, const double& yArg, const double& radiusArg, const std::string& labelArg ) : x( xArg ), y( yArg ), radius( radiusArg ), label( labelArg ) {

}

// Deconstructor
Obstacle::~Obstacle() {}
