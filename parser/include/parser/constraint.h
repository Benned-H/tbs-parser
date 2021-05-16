// Implements a TBS constraint, which is a union of location relations
// Author: Benned Hedegaard

#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include <vector>
#include "parser/location-relation.h"

class Constraint {
	public:
	
	    Constraint( const std::vector<LocationRelation>& relationsArg ); // Constructor
	    Constraint( const Constraint& c ); // Copy constructor
		virtual ~Constraint(); // Deconstructor
		
		std::vector<LocationRelation> relations;
};

#endif /* CONSTRAINT_H */
