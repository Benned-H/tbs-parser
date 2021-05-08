// Implements a circular obstacle
// Author: Benned Hedegaard

#include "planner/obstacle.h"

// Constructor
Obstacle::Obstacle( const double& xArg, const double& yArg, const double& radiusArg, const ObstacleType& typeArg, const uint32_t& numberArg ) : x( xArg ), y( yArg ), radius( radiusArg ), type( typeArg ), number( numberArg ) {

}

// Deconstructor
Obstacle::~Obstacle() {}

// Returns a message representation of this object
planner::ObstacleMsg Obstacle::to_msg( void ) const {
    planner::ObstacleMsg o;
    o.x.data = x;
    o.y.data = y;
    o.radius.data = radius;
    o.type.data = ( uint16_t ) type;
    o.number.data = number;
    return o;
}
