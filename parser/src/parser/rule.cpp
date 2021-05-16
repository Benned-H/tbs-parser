// Implements a Context-Free Grammar rule, i.e. a set of productions
// Author: Benned Hedegaard

#include <stdlib.h>
#include "parser/rule.h"

// Empty constructor
Rule::Rule( void ) : productions() {

}

// Deconstructor
Rule::~Rule() {}

// Assignment operator
Rule& Rule::operator=( const Rule& r ) {
    productions.clear(); // Clear my productions
    for ( int i = 0; i < r.productions.size(); i++ ) {
        productions.push_back( r.productions[i] ); // Copy r's productions
    }
}
