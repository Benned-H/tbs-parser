// Implements the Homotopy-Aware RRT* planner introduced by Yi et al. (2016)
// Author: Benned Hedegaard

#ifndef HARRT_H
#define HARRT_H

#include <vector>
#include "ros/ros.h"
#include "planner/ReferenceFrameArrayMsg.h"
#include "planner/reference-frame.h"
#include "planner/world-model.h"
#include "planner/action.h"
#include "planner/path.h"
#include "planner/rrt.h"

class HomotopyAwareRRT {

	public:
	
		HomotopyAwareRRT( const int& iterationsArg ); // Constructor
		virtual ~HomotopyAwareRRT(); // Deconstructor
		
		void handleWorldModel( const planner::WorldModelMsg::ConstPtr& msg );
		
		ros::Publisher reference_frames_pub;
		
		// Publishes the current stored reference frames
		void publishFrames( void );
		
		// Finds the reference frames for a particular world and obstacle c_i of interest
		std::vector<ReferenceFrame> decomposeWorld( const int& c_i ) const;
		
		//  Plans an optimal path from x_start to x_goal obeying a given homotopy specification
		std::vector<Path> search( const Point& x_start, const Point& x_goal, const Action& a );
		
		/*
            TODO
            Explore - Expands the tree T by randomly sampling a new node and attempting to connect it to the tree.
            Input: RRT T, integer i
            Output: Newly sampled node x_new

            CRF - Returns the ID characters representing the crossed reference frames of line segment l.
            Input: Line segment L
            Output: Crossed reference frames as a string

            STR - Returns the string representing the crossed reference frames from the root of T to x, in order.
            Input: Node x, Tree T
            Output: String v

            StringCheck - Checks if the given string meets the current homotopy constraints.
            Input: String v, constraints C
            Output: Boolean yes/no

            Connect - Connects the node x_new to RRT T.
            Input: x_new, T
            Output: Minimum ??? p_min

            Path - Returns the path from the root of the tree T to the vertex v.
            Input: Vertex v, Tree T
            Output: Path P

            Concat - Concatenates the two given paths.
            Input: Paths a and b
            Output: Concatenated path ab

            MergePaths - Applies REPTrim() in order to merge string blocks into homotopy classes.
            Input - Stored string blocks SB
            Output - Simplified set of string blocks SB'
		*/
		
		bool has_world_model;
		
	protected:
	
		WorldModel wm;
		std::vector<ReferenceFrame> frames;
		
		int num_iterations;
};

#endif /* HARRT_H */
