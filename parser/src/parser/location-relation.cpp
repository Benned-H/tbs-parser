// Implements a TBS constraint, which is a union of location relations
// Author: Benned Hedegaard

#include "parser/location-relation.h"

// Constructor
LocationRelation::LocationRelation( const RelationType& typeArg, const bool& negatedArg ) : type( typeArg ), negated( negatedArg ) {

}

// Copy constructor
LocationRelation::LocationRelation( const LocationRelation& lr ) : type( lr.type ), negated( lr.negated ) {

}

// Deconstructor
LocationRelation::~LocationRelation() {}
