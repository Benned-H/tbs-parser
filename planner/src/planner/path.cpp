// Implements a path over Nodes
// Author: Benned Hedegaard

#include "planner/path.h"

// Constructor
Path::Path( const bool& forwardArg ) : forward( forwardArg ), cost( 0.0 ), str( "" ), nodes() {

}

// Copy constructor
Path::Path( const Path& p ) : forward( p.forward ), cost( p.cost ), str( p.str ), nodes( p.nodes ) {

}

// Deconstructor
Path::~Path() {}

// Assignment operator
Path& Path::operator=( const Path& p ) {
    forward = p.forward;
    cost = p.cost;
    str = p.str;
    
    nodes.clear();
    for ( int i = 0; i < p.nodes.size(); i++ ) {
        nodes.push_back( p.nodes[i] );
    }
}
