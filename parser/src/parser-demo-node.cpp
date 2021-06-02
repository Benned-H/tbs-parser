// This node tests the parser's ability to:
//      1) Read a CFG in BNF from a file
//      2) Sample random strings from a CFG
//      3) Parse strings using a read-from-file CFG
// Author: Benned Hedegaard

#include "ros/ros.h"
#include "parser/cfg.h"

int main( int argc, char* argv[] ) {
    if ( argc < 3 ) { // We expect at least two args from the launch file
        std::cout << "argc is " << argc << std::endl;
        std::cout << "Expected two args: tbs_file and sample_num" << std::endl;
        return 0;
    }

	ros::init( argc, argv, "parser_demo_node" );
	ros::NodeHandle node_handle;
	
	sleep(1); // Wait 1 second; gives time for ROS connections to be made.
	
	// Construct CFG for the TBS grammar by reading from file
	CFG tbs;
	tbs.from_file( argv[1] );
	
	/*
	std::cout << "Created CFG for the TBS. Populating now..." << std::endl;
    
    // Sort the terminal and nonterminal lists to make duplicates obvious
    std::sort( tbs.terminals.begin(), tbs.terminals.end() );
    std::sort( tbs.nonterminals.begin(), tbs.nonterminals.end() );
    
    std::cout << "Created TBS grammar:" << std::endl;
    std::cout << tbs << std::endl;
    
    /*
	int seed = std::stoi( argv[1] );
	std::cout << "Seed given was " << seed << std::endl;
	
	int num_samples = std::stoi( argv[2] );
	std::cout << "Number of samples given was " << num_samples << std::endl;
    
    
    // Now let's generate some example strings!
    int number_samples = 5;
    int max_length = 10;
    
    srand( 282 ); // Seed RNG for sampling productions
    
    std::cout << "Generating " << number_samples << " strings using the grammar..." << std::endl;
    std::vector<std::string> results = tbs.sample_strings( number_samples, max_length );
    for ( std::string& str : results ) {
        std::cout << str << std::endl;
    }*/
	
	return 0;
}
