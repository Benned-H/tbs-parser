// Implements a node for the RRT* algorithm
// Author: Benned Hedegaard

#include "planner/node.h"

// Constructor
Node::Node( const double& xArg, const double& yArg ) : x( xArg ), y( yArg ), cost( 0.0 ), parent( nullptr ), children(), str( "" ) {

}

// Copy constructor
Node::Node( const Node& n ) : x( n.x ), y( n.y ), cost( n.cost ), parent( n.parent ), children( n.children ), str( n.str ) {

}

// Deconstructor
Node::~Node() {}

// Assignment operator
Node& Node::operator=( const Node& n ) {
    x = n.x;
    y = n.y;
    parent = n.parent;
    children = n.children;
    cost = n.cost;
    str = n.str;
}

// Equality operator
bool Node::operator==( const Node& n ) const {
    return ( ( n.x == x ) && ( n.y == y ) );
}
