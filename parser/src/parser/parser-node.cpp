// Tests the parser functionality
// Author: Benned Hedegaard

#include "ros/ros.h"
#include "parser/cfg.h"

int main( int argc, char* argv[] ) {

	ros::init( argc, argv, "parser_node" );
	ros::NodeHandle node_handle;
	
	sleep(1); // Wait 1 second; gives time for ROS connections to be made.
	
	// Construct CFG for the TBS grammar
	CFG tbs;
	
	std::cout << "Created CFG for the TBS. Populating now..." << std::endl;
	
	// TODO - Manually creating these rules is lazy; writing file i/o in C++ would be arduous at this point. Fix later!
	
	// <tbs> ::= navigate <mode> <constraint-list> to the <object> <location-relation>
	std::shared_ptr<Rule> tbs_rule = std::make_shared<Rule>();
	std::vector<std::string> tbs_prod{ "navigate", "<mode>", "<constraint-list>", "to", "the", "<object>", "<goal-relation>" };
	tbs_rule->productions.push_back( std::make_shared<std::vector<std::string>>( tbs_prod ) );
	
	// Add new terminals to the CFG
	tbs.add_terminal( "navigate" );
	tbs.add_terminal( "to" );
	tbs.add_terminal( "the" );
	
	// Add new nonterminals to the CFG
	tbs.add_nonterminal( "<tbs>" );
    tbs.add_nonterminal( "<mode>" );
    tbs.add_nonterminal( "<constraint-list>" );
    tbs.add_nonterminal( "<object>" );
    tbs.add_nonterminal( "<goal-relation>" );
    
    // Also set start symbol
    tbs.start_symbol = "<tbs>";
    
    // Add rule to the grammar
    tbs.rules["<tbs>"] = tbs_rule;
	
	// <mode> ::= quickly | safely | <epsilon>
	std::shared_ptr<Rule> mode_rule = std::make_shared<Rule>();
	std::vector<std::string> mode_prod1{ "quickly" };
	std::vector<std::string> mode_prod2{ "safely" };
	std::vector<std::string> mode_prod3{ "<epsilon>" };
	mode_rule->productions.push_back( std::make_shared<std::vector<std::string>>( mode_prod1 ) );
	mode_rule->productions.push_back( std::make_shared<std::vector<std::string>>( mode_prod2 ) );
	mode_rule->productions.push_back( std::make_shared<std::vector<std::string>>( mode_prod3 ) );
	
	// Add new terminals to the CFG
	tbs.add_terminal( "quickly" );
	tbs.add_terminal( "safely" );
	
	// Add new nonterminals to the CFG
	tbs.add_nonterminal( "<epsilon>" );
	
	// Add rule to the grammar
    tbs.rules["<mode>"] = mode_rule;
	
	// <constraint-list> ::= <relation-list> <constraint-tail> | <epsilon>
	std::shared_ptr<Rule> constraint_list_rule = std::make_shared<Rule>();
	std::vector<std::string> constraint_list_prod1{ "<relation-list>", "<constraint-tail>" };
	std::vector<std::string> constraint_list_prod2{ "<epsilon>" };
	constraint_list_rule->productions.push_back( std::make_shared<std::vector<std::string>>( constraint_list_prod1 ) );
	constraint_list_rule->productions.push_back( std::make_shared<std::vector<std::string>>( constraint_list_prod2 ) );
	
	// Add new nonterminals to the CFG
	tbs.add_nonterminal( "<relation-list>" );
	tbs.add_nonterminal( "<constraint-tail>" );
	
	// Add rule to the grammar
    tbs.rules["<constraint-list>"] = constraint_list_rule;
	
	// <constraint-tail> ::= then <relation-list> <constraint-tail> | <epsilon>
	std::shared_ptr<Rule> constraint_tail_rule = std::make_shared<Rule>();
	std::vector<std::string> constraint_tail_prod1{ "then", "<relation-list>", "<constraint-tail>" };
	std::vector<std::string> constraint_tail_prod2{ "<epsilon>" };
	constraint_tail_rule->productions.push_back( std::make_shared<std::vector<std::string>>( constraint_tail_prod1 ) );
	constraint_tail_rule->productions.push_back( std::make_shared<std::vector<std::string>>( constraint_tail_prod2 ) );
	
	// Add new terminals to the CFG
	tbs.add_terminal( "then" );
	
	// Add rule to the grammar
    tbs.rules["<constraint-tail>"] = constraint_tail_rule;
	
	// <relation-list> ::= <relation> <relation-tail>
	std::shared_ptr<Rule> relation_list_rule = std::make_shared<Rule>();
	std::vector<std::string> relation_list_prod{ "<relation>", "<relation-tail>" };
	relation_list_rule->productions.push_back( std::make_shared<std::vector<std::string>>( relation_list_prod ) );
	
	// Add new nonterminals to the CFG
	tbs.add_nonterminal( "<relation>" );
	tbs.add_nonterminal( "<relation-tail>" );
	
	// Add rule to the grammar
    tbs.rules["<relation-list>"] = relation_list_rule;
	
	// <relation-tail> ::= or <relation> <relation-tail> | <epsilon>
	std::shared_ptr<Rule> relation_tail_rule = std::make_shared<Rule>();
	std::vector<std::string> relation_tail_prod1{ "or", "<relation>", "<relation-tail>" };
	std::vector<std::string> relation_tail_prod2{ "<epsilon>" };
	relation_tail_rule->productions.push_back( std::make_shared<std::vector<std::string>>( relation_tail_prod1 ) );
	relation_tail_rule->productions.push_back( std::make_shared<std::vector<std::string>>( relation_tail_prod2 ) );
	
	// Add new terminals to the CFG
	tbs.add_terminal( "or" );
	
	// Add rule to the grammar
    tbs.rules["<relation-tail>"] = relation_tail_rule;
	
	// <relation> ::= <location-relation> | not <location-relation>
	std::shared_ptr<Rule> relation_rule = std::make_shared<Rule>();
	std::vector<std::string> relation_prod1{ "<location-relation>" };
	std::vector<std::string> relation_prod2{ "not", "<location-relation>" };
	relation_rule->productions.push_back( std::make_shared<std::vector<std::string>>( relation_prod1 ) );
	relation_rule->productions.push_back( std::make_shared<std::vector<std::string>>( relation_prod2 ) );
	
	// Add new terminals to the CFG
	tbs.add_terminal( "not" );
	
	// Add new nonterminals to the CFG
	tbs.add_nonterminal( "<location-relation>" );
	
	// Add rule to the grammar
    tbs.rules["<relation>"] = relation_rule;
	
	// <goal-relation> ::= <location-relation> | <epsilon>
	std::shared_ptr<Rule> goal_relation_rule = std::make_shared<Rule>();
	std::vector<std::string> goal_relation_prod1{ "<location-relation>" };
	std::vector<std::string> goal_relation_prod2{ "<epsilon>" };
	goal_relation_rule->productions.push_back( std::make_shared<std::vector<std::string>>( goal_relation_prod1 ) );
	goal_relation_rule->productions.push_back( std::make_shared<std::vector<std::string>>( goal_relation_prod2 ) );
	
	// Add rule to the grammar
    tbs.rules["<goal-relation>"] = goal_relation_rule;
	
	// <location-relation> ::= left of the <object> | right of the <object> | behind the <object> | 
	//          in front of the <object> | near the <object> | far from the <object> | between the <object> and <object>
	std::shared_ptr<Rule> location_relation_rule = std::make_shared<Rule>();
	std::vector<std::string> location_relation_prod1{ "left", "of", "the", "<object>" };
	std::vector<std::string> location_relation_prod2{ "right", "of", "the", "<object>" };
	std::vector<std::string> location_relation_prod3{ "behind", "the", "<object>" };
	std::vector<std::string> location_relation_prod4{ "in", "front", "of", "the", "<object>" };
	std::vector<std::string> location_relation_prod5{ "near", "the", "<object>" };
	std::vector<std::string> location_relation_prod6{ "far", "from", "the", "<object>" };
	std::vector<std::string> location_relation_prod7{ "between", "the", "<object>", "and", "<object>" };
	location_relation_rule->productions.push_back( std::make_shared<std::vector<std::string>>( location_relation_prod1 ) );
	location_relation_rule->productions.push_back( std::make_shared<std::vector<std::string>>( location_relation_prod2 ) );
	location_relation_rule->productions.push_back( std::make_shared<std::vector<std::string>>( location_relation_prod3 ) );
	location_relation_rule->productions.push_back( std::make_shared<std::vector<std::string>>( location_relation_prod4 ) );
	location_relation_rule->productions.push_back( std::make_shared<std::vector<std::string>>( location_relation_prod5 ) );
	location_relation_rule->productions.push_back( std::make_shared<std::vector<std::string>>( location_relation_prod6 ) );
	location_relation_rule->productions.push_back( std::make_shared<std::vector<std::string>>( location_relation_prod7 ) );
	
	// Add new terminals to the CFG
	tbs.add_terminal( "left" );
	tbs.add_terminal( "of" );
	tbs.add_terminal( "right" );
	tbs.add_terminal( "behind" );
	tbs.add_terminal( "in" );
	tbs.add_terminal( "front" );
	tbs.add_terminal( "near" );
	tbs.add_terminal( "far" );
	tbs.add_terminal( "from" );
	tbs.add_terminal( "between" );
	tbs.add_terminal( "and" );
	
	// Add rule to the grammar
    tbs.rules["<location-relation>"] = location_relation_rule;
	
	// <object> ::= barrel | bush | cone | hydrant
	std::shared_ptr<Rule> object_rule = std::make_shared<Rule>();
	std::vector<std::string> object_prod1{ "barrel" };
	std::vector<std::string> object_prod2{ "bush" };
	std::vector<std::string> object_prod3{ "cone" };
	std::vector<std::string> object_prod4{ "hydrant" };
	object_rule->productions.push_back( std::make_shared<std::vector<std::string>>( object_prod1 ) );
	object_rule->productions.push_back( std::make_shared<std::vector<std::string>>( object_prod2 ) );
	object_rule->productions.push_back( std::make_shared<std::vector<std::string>>( object_prod3 ) );
	object_rule->productions.push_back( std::make_shared<std::vector<std::string>>( object_prod4 ) );
	
	// Add new terminals to the CFG
	tbs.add_terminal( "barrel" );
	tbs.add_terminal( "bush" );
	tbs.add_terminal( "cone" );
	tbs.add_terminal( "hydrant" );
	
	// Add rule to the grammar
    tbs.rules["<object>"] = object_rule;
    
    // Sort the terminal and nonterminal lists to make duplicates obvious
    std::sort( tbs.terminals.begin(), tbs.terminals.end() );
    std::sort( tbs.nonterminals.begin(), tbs.nonterminals.end() );
    
    std::cout << "Created TBS grammar:" << std::endl;
    std::cout << tbs << std::endl;
    
    // Now let's generate some example strings!
    int number_samples = 5;
    int max_length = 10;
    
    srand( 282 ); // Seed RNG for sampling productions
    
    std::cout << "Generating " << number_samples << " strings using the grammar..." << std::endl;
    std::vector<std::string> results = tbs.sample_strings( number_samples, max_length );
    for ( std::string& str : results ) {
        std::cout << str << std::endl;
    }
	
	return 0;
}
