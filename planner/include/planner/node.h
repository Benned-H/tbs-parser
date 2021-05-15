// Implements a node for the RRT* algorithm
// Author: Benned Hedegaard

#ifndef NODE_H
#define NODE_H

#include <vector>
#include <string>
#include <memory>

class Node {
	public:
	
	    Node( const double& xArg, const double& yArg ); // Constructor
	    Node( const Node& n ); // Copy constructor
		virtual ~Node(); // Deconstructor
		Node& operator=( const Node& n ); // Assignment operator
		bool operator==( const Node& n ) const; // Equality operator
		
		double x;
		double y;
		double cost;
		
		std::shared_ptr<Node> parent;
		std::vector<std::shared_ptr<Node>> children;
		std::string str;
};

#endif /* NODE_H */
