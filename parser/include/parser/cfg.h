// Implements a generic Context-Free Grammar
// Author: Benned Hedegaard

#ifndef CFG_H
#define CFG_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <memory>
#include <unordered_map>
#include "parser/rule.h"
#include "parser/command.h"

class CFG {
	public:
	
	    CFG( void ); // Constructor
		virtual ~CFG(); // Deconstructor
		
		// Reads and populates the CFG from a file in Backus-Naur Form
		void from_file( const std::string& filename );
		
		// Adds a new production to the Rule for the given nonterminal
		void add_production( const std::string& nt, const std::shared_ptr<std::vector<std::string>>& production );
		
		// Adds a new terminal to the CFG
		void add_terminal( const std::string& t );
		
		// Returns whether the given string is already a terminal in the CFG
		bool is_terminal( const std::string& t ) const;
		
		// Adds a new nonterminal to the CFG
		void add_nonterminal( const std::string& nt );
		
		// Returns whether the given string is already a nonterminal in the CFG
		bool is_nonterminal( const std::string& nt ) const;
		
		// Tokenizes the given string based on spaces
		std::vector<std::string> tokenize_on_spaces( const std::string& str ) const;
		
		// Returns a random production for the given nonterminal
        std::shared_ptr<std::vector<std::string>> get_rand_production( const std::string& nt ) const;
		
		// Samples the given number of strings from the grammar, rejecting any longer than max_length
        std::vector<std::string> sample_strings( const int& num, const int& max_length ) const;
        
        // Attempts to parse the given string into a Command object
        std::shared_ptr<Command> parse( const std::string& str ) const;
		
		// Member variables
		std::string start_symbol;
		std::vector<std::string> nonterminals;
		std::vector<std::string> terminals;
		std::unordered_map<std::string, std::shared_ptr<Rule>> rules;
};

std::ostream& operator<<( std::ostream& os, const CFG& cfg );

#endif /* CFG_H */
