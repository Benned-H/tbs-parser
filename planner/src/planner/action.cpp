// Implements a TBS action, which is one navigate command under the grammar
// Author: Benned Hedegaard

#include "planner/action.h"

// Constructor
Action::Action( const Mode& modeArg, const std::vector<Constraint>& motionArgs, const ObstacleType& goalArg, const std::vector<Constraint>& goalArgs ) : mode( modeArg ), motion_constraints( motionArgs ), goal( goalArg ), goal_constraints( goalArgs ) {

}

// Copy constructor
Action::Action( const Action& a ) : mode( a.mode ), motion_constraints( a.motion_constraints ), goal( a.goal ), goal_constraints( a.goal_constraints ) {

}

// Deconstructor
Action::~Action() {}
