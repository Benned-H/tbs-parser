// Implements a TBS action, which is one navigate command under the grammar
// Author: Benned Hedegaard

#ifndef ACTION_H
#define ACTION_H

#include "planner/constraint.h"

enum Mode { QUICK, SAFE, NONE };
enum ObstacleType { BARREL, BUSH, CONE, HYDRANT };

class Action {
	public:
	
	    Action( const Mode& modeArg, const std::vector<Constraint>& motionArgs, const ObstacleType& goalArg, const std::vector<Constraint>& goalArgs ); // Constructor
	    Action( const Action& a ); // Copy constructor
		virtual ~Action(); // Deconstructor
		
		// Properties of the action are its mode, motion constraints, goal, and goal constraints
		Mode mode;
		std::vector<Constraint> motion_constraints; // Can be empty
		ObstacleType goal;
		std::vector<Constraint> goal_constraints; // Can be empty
};

#endif /* ACTION_H */
