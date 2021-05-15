// Implements a TBS navigate command
// Author: Benned Hedegaard

#ifndef COMMAND_H
#define COMMAND_H

#include "planner/constraint.h"

enum Mode { QUICK, SAFE, NONE };
enum ObstacleType { BARREL, BUSH, CONE, HYDRANT };

class Command {
	public:
	    
	    Command( void ); // Empty constructor
	    Command( const Mode& modeArg, const std::vector<Constraint>& motionArgs, const ObstacleType& goalArg, const std::vector<LocationRelation>& goalArgs ); // Constructor
	    Command( const Command& c ); // Copy constructor
		virtual ~Command(); // Deconstructor
		
		// Properties of the command are its mode, motion constraints, goal, and goal constraints
		Mode mode;
		std::vector<Constraint> motion_constraints; // Can be empty
		ObstacleType goal;
		std::vector<LocationRelation> goal_constraints; // Can be empty
};

#endif /* COMMAND_H */
