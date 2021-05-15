// Implements a path over Nodes
// Author: Benned Hedegaard

#ifndef PATH_H
#define PATH_H

#include <vector>
#include <memory>
#include "planner/node.h"

class Path {
	public:
	
	    Path( const bool& forwardArg ); // Constructor
	    Path( const Path& p ); // Copy constructor
		virtual ~Path(); // Deconstructor
		Path& operator=( const Path& p ); // Assignment operator
		
		bool forward; // Indicates whether the Path is forward or backward w.r.t. x_start
		double cost; // Cost of the Path
		std::string str; // Homotopy string of the Path
		std::vector<std::shared_ptr<Node>> nodes;
};

#endif /* PATH_H */
