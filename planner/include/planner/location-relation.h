// Implements a TBS location relation, which is one of seven options
// Author: Benned Hedegaard

#ifndef LOCATION_RELATION_H
#define LOCATION_RELATION_H

enum RelationType { LEFT, RIGHT, BEHIND, FRONT, NEAR, FAR, BETWEEN };

class LocationRelation {
	public:
	
	    LocationRelation( const RelationType& typeArg, const bool& negatedArg ); // Constructor
	    LocationRelation( const LocationRelation& lr ); // Copy constructor
		virtual ~LocationRelation(); // Deconstructor
		
		RelationType type; // One of the seven options enumerated above
		bool negated; // Indicates if this relation is negated in the command	
};

#endif /* LOCATION_RELATION_H */
