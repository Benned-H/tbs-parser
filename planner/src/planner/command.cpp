// Implements a TBS navigate command
// Author: Benned Hedegaard

#include "planner/command.h"

// Empty constructor
Command::Command( void ) : mode( Mode::NONE ), motion_constraints(), goal( ObstacleType::BARREL ), goal_constraints() {

}

// Constructor
Command::Command( const Mode& modeArg, const std::vector<Constraint>& motionArgs, const ObstacleType& goalArg, const std::vector<LocationRelation>& goalArgs ) : mode( modeArg ), motion_constraints( motionArgs ), goal( goalArg ), goal_constraints( goalArgs ) {

}

// Copy constructor
Command::Command( const Command& c ) : mode( c.mode ), motion_constraints( c.motion_constraints ), goal( c.goal ), goal_constraints( c.goal_constraints ) {

}

// Deconstructor
Command::~Command() {}
