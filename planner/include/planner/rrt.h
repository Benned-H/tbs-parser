// Implements the RRT* tree for the HARRT* planner
// Author: Benned Hedegaard

#ifndef RRT_H
#define RRT_H

#include <vector>
//#include <utility>

class RRT {

	public:
	
		RRT( void ); // Empty constructor
		virtual ~RRT(); // Deconstructor
		
		// Expands the tree by randomly sampling a new node and attempting to connect it to the tree
		Point explore( const int& i );
		
		std::vector<Point> nodes;
		std::vector<std::pair<Point,Point>> edges;
};

#endif /* RRT_H */
