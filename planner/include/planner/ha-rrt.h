// Implements the Homotopy-Aware RRT* planner introduced by Yi et al. (2016)
// Author: Benned Hedegaard

#ifndef HARRT_H
#define HARRT_H

#include <stdlib.h> // Use C rand() and srand() functions
#include <vector>
#include <memory>
#include <string>
#include <unordered_map>
#include "ros/ros.h"
#include "planner/ObstacleMsg.h"
#include "planner/ReferenceFrameArrayMsg.h"
#include "planner/reference-frame.h"
#include "planner/world-model.h"
#include "parser/command.h"
#include "planner/path.h"
#include "planner/node.h"

class HomotopyAwareRRT {

	public:
	
		HomotopyAwareRRT( const int& iterationsArg, const double& etaArg ); // Constructor
		virtual ~HomotopyAwareRRT(); // Deconstructor
		
		void handleWorldModel( const planner::WorldModelMsg::ConstPtr& msg );
		
		ros::Publisher reference_frames_pub;
		
		// Publishes the current stored reference frames
		void publishFrames( void );
		
		// Finds the reference frames for a particular world and obstacle c_i of interest
		std::vector<ReferenceFrame> decomposeWorld( const int& c_i ) const;
		
		// Computes gamma (used in HARRT* search) based on the world model
		void update_gamma( void );
		
		//  Plans an optimal path from x_start to x_goal obeying the stored homotopy command
		std::unordered_map<std::string, Path> search( const std::shared_ptr<Node>& x_start, const std::shared_ptr<Node>& x_goal );
		
		// Expands the tree by randomly sampling a new node and attempting to connect it to the tree
		std::shared_ptr<Node> explore( std::vector<std::shared_ptr<Node>>& T, const int& i, const bool& forward );
		
		// Returns the path constructed by connecting Node x_new to tree T
		Path connect( const std::shared_ptr<Node>& x_new, const std::vector<std::shared_ptr<Node>>& T, const bool& x_new_forward ) const;
		
		// Returns whether the given point falls inside an obstacle in the current world model
        bool inside_obstacle( const double& x, const double& y ) const;
        
        // Randomly samples a Node from obstacle-free configuration space
        std::shared_ptr<Node> sample( const int& i ) const;
        
        // Steers from the first Node toward the second Node a distance of eta
        std::shared_ptr<Node> steer( const std::shared_ptr<Node>& n1, const std::shared_ptr<Node>& n2 ) const;
        
        // Returns the Nodes in the given tree within radius r of the given Node
        std::vector<std::shared_ptr<Node>> near( const std::vector<std::shared_ptr<Node>>& T, const std::shared_ptr<Node>& n ) const;
        
        // Checks if the line segment between the two Nodes lies in obstacle-free space
        bool obstacle_free( const std::shared_ptr<Node>& n1, const std::shared_ptr<Node>& n2 ) const;
        
        // Returns the crossed reference frames on the line from Node n1 to Node n2
        std::string crf( const std::shared_ptr<Node>& n1, const std::shared_ptr<Node>& n2 ) const;
        
        // Checks if v is a substring of the current homotopy command
        // The forward argument allows us to handle the backward tree correctly
        bool string_check( const std::string& v, const bool& forward ) const;
        
        // Returns the Euclidean distance between the given (x,y) points
        double distance( const double& x1, const double& y1, const double& x2, const double& y2 ) const;
        
        // Returns the cost of the line between the two nodes
        double c_line( const std::shared_ptr<Node>& n1, const std::shared_ptr<Node>& n2 ) const;

        // Returns the closest Node to the given Node in the given tree
        std::shared_ptr<Node> nearest( const std::vector<std::shared_ptr<Node>>& T, const std::shared_ptr<Node>& n ) const;
        
        bool has_world_model;
        
        // Member variables used for the HARRT* search process
		int num_iterations;
		double eta; // Bounds the distance one sample can steer when building an RRT*
		double gamma; // Used to ensure optimality of HARRT*
		
	protected:
	
		WorldModel wm; // Model of the world composed of obstacles and world boundaries
		std::vector<ReferenceFrame> frames; // Reference frames corresponding to current world model
		
		Command command; // Current homotopy constraint command
		// TODO - Somehow map command to strings to populate command_homotopy_classes. Use REPTrim or ensure nonrepeating!
		std::vector<std::string> command_homotopy_classes; // All homotopy strings satisfying the current command
};

#endif /* HARRT_H */
