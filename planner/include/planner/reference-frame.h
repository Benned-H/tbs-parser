// Implements a reference frame, which is a parameterized line segment and label
// Author: Benned Hedegaard

#ifndef REFERENCE_FRAME_H
#define REFERENCE_FRAME_H

#include "planner/point.h"
#include "planner/line.h"
#include "planner/ReferenceFrameMsg.h"

class ReferenceFrame {
	public:
	
		ReferenceFrame( const char& idArg, const Point& p1Arg, const Point& p2Arg ); // Constructor
		virtual ~ReferenceFrame(); // Deconstructor
		
		planner::ReferenceFrameMsg to_msg( void ) const;
		
		// We parameterize a line segment using some functions x(t) and y(t)
		// We range from t = 0 at (xi,yi) to t = 1 at (xf,yf)
		// Then x(t) = xi + ( xf - xi ) * t, 0 < t < 1
		//      y(t) = yi + ( yf - yi ) * t, 0 < t < 1
		
		char id; // Reference frame ID
		Line l; // Line contains two points
};

std::ostream& operator<<( std::ostream& os, const ReferenceFrame& r );

#endif /* REFERENCE_FRAME_H */
