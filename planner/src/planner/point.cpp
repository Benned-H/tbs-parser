// Implements an (x,y) point
// Author: Benned Hedegaard

#include <cmath>
#include "planner/point.h"

// Constructor
Point::Point( const double& xArg, const double& yArg, const int& obsArg ) : x( xArg ), y( yArg ), obstacle( obsArg ) {

}

// Copy constructor
Point::Point( const Point& p ) : x( p.x ), y( p.y ) {

}

// Deconstructor
Point::~Point() {}

planner::PointMsg Point::to_msg( void ) const {
    planner::PointMsg msg;
    msg.x = x;
    msg.y = y;
    msg.o = obstacle;
    return msg;
}

// Returns the Euclidean distance between this and another Point
double Point::distance( const Point& p ) const {
    return std::sqrt( ( x - p.x ) * ( x - p.x ) + ( y - p.y ) * ( y - p.y ) );
}

// Returns the Euclidean distance between this and an (x,y) coordinate
double Point::distance( const double& x1, const double& y1 ) const {
    return std::sqrt( ( x - x1 ) * ( x - x1 ) + ( y - y1 ) * ( y - y1 ) );
}

// Returns whether this Point falls onto the same line as the two given Points
bool Point::same_line( const Point& p1, const Point& p2 ) const {
    if ( p1.x == p2.x ) { // Handle vertical line case
        return ( p1.x == x );
    }
    
    // Otherwise we consider a point-slope line
    return ( ( y - p1.y ) == ( ( x - p1.x ) * ( p2.y - p1.y ) / ( p2.x - p1.x ) ) );
}

bool compare_x( const Point& p1, const Point& p2 ) {
    return (p1.x < p2.x );
}

std::ostream& operator<<( std::ostream& os, const Point& p ) {
    os << "Point (" << p.x << "," << p.y << ") from obstacle " << p.obstacle;
    return os;
}
