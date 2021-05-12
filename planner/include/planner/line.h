// Implements a line based on two points
// Author: Benned Hedegaard

#ifndef LINE_H
#define LINE_H

#include <vector>
#include "planner/point.h"

class Line {
	public:
	
	    Line( const Point& p1Arg, const Point& p2Arg ); // Constructor
	    Line( const Line& l ); // Copy constructor
		virtual ~Line(); // Deconstructor
		
		// Returns a vector containing the points at which this line intersects the given circle
		std::vector<Point> intersect_circle( const double& xc, const double& yc, const double& rc, const int& obstacle ) const;
		
		// Returns a vector containing point at which this line intersects the given line
		std::vector<Point> intersect_line( const Line& l ) const;
		
		Point p1;
		Point p2;
};

std::ostream& operator<<( std::ostream& os, const Line& l );

#endif /* LINE_H */
