// Implements an (x,y) point
// Author: Benned Hedegaard

#ifndef POINT_H
#define POINT_H

#include "planner/PointMsg.h"

class Point {
	public:
	
	    Point( const double& x, const double& y, const int& obsArg = -1 ); // Constructor
	    Point( const Point& p ); // Copy constructor
		virtual ~Point(); // Deconstructor
		
		bool operator < ( const Point& p );
		
		planner::PointMsg to_msg( void ) const;
		
		// Returns the Euclidean distance between this and another Point
		double distance( const Point& p ) const;
		
		// Returns the Euclidean distance between this and an (x,y) coordinate
		double distance( const double& x1, const double& y1 ) const;
		
		// Returns whether this Point falls onto the same line as the two given Points
        bool same_line( const Point& p1, const Point& p2 ) const;
		
		double x;
		double y;
		int obstacle; // Obstacle index associated with this Point. -1 is a wall
};

bool compare_x( const Point& p1, const Point& p2 );

std::ostream& operator<<( std::ostream& os, const Point& p );

#endif /* POINT_H */
