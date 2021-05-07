// Implements the Homotopy-Aware RRT* planner introduced by Yi et al. (2016)
// Author: Benned Hedegaard

#ifndef HARRT_H
#define HARRT_H

// TODO - Include all (standard) library dependencies
//#include <iostream>
//#include <vector>
//#include <Eigen/Dense>

// TODO - Include all necessary header files
//#include "ros/ros.h"
//#include "package_name/DataType.h" // etc.

class HomotopyAwareRRT {

	public:
	
		HomotopyAwareRRT(); // Constructor
		virtual ~HomotopyAwareRRT(); // Deconstructor
		
		// TODO - Declare message handling functions for the class.
		//void handleMessageType( const package_name::DataType::ConstPtr& msg );
		
		// TODO - Declare any ROS publishers.
		//ros::Publisher datatype_pub;
		
	protected:
	
		// TODO - Include any internal functions the class needs.
		
		// TODO - Private member variables. Start names with underscores.
		package_name::DataType _data;
		bool _condition;
};

#endif /* HARRT_H */
