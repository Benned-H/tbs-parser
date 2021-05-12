// Implements a path over points
// Author: Benned Hedegaard

#include "planner/path.h"

// Constructor
Path::Path( const bool& reversedArg, const std::vector<Point>& pointsArg ) : reversed( reversedArg ), points( pointsArg ) {

}

// Copy constructor
Path::Path( const Path& p ) : reversed( p.reversed ), points( p.points ) {

}

// Deconstructor
Path::~Path() {}
