// Implements a path over points
// Author: Benned Hedegaard

#ifndef PATH_H
#define PATH_H

#include <vector>
#include "planner/point.h"

class Path {
	public:
	
	    Path( const bool& reversedArg, const std::vector<Point>& pointsArg ); // Constructor
	    Path( const Path& p ); // Copy constructor
		virtual ~Path(); // Deconstructor
		
		bool reversed;
		std::vector<Point> points;
};

#endif /* PATH_H */
