// Implements a TBS constraint, which is a union of location relations
// Author: Benned Hedegaard

#include "parser/constraint.h"

// Constructor
Constraint::Constraint( const std::vector<LocationRelation>& relationsArg ) : relations( relationsArg ) {

}

// Copy constructor
Constraint::Constraint( const Constraint& c ) : relations( c.relations ) {

}

// Deconstructor
Constraint::~Constraint() {}
