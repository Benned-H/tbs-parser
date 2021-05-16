// Implements a generic Context-Free Grammar
// Author: Benned Hedegaard

#include <stdlib.h>
#include <iostream>
#include "parser/cfg.h"

/*** Helper functions ***/

bool in( const std::string& str, const std::vector<std::string>& vec ) {
    for ( int i = 0; i < vec.size(); i++ ) {
        if ( str.compare( vec[i] ) == 0 ) {
            return true;
        }
    }
    return false;
}

/*** End helper functions ***/

// Constructor
CFG::CFG( void ) : start_symbol( "" ), nonterminals(), terminals(), rules() {
    
}

// Deconstructor
CFG::~CFG() {}

// Adds a new terminal to the CFG
void CFG::add_terminal( const std::string& t ) {
    terminals.push_back( t );
}

// Adds a new nonterminal to the CFG
void CFG::add_nonterminal( const std::string& nt ) {
    nonterminals.push_back( nt );
    std::shared_ptr<Rule> empty_rule = std::make_shared<Rule>(); // Create empty rule for this nt
    rules.insert({ nt, empty_rule });
}

// Returns a random production for the given nonterminal
std::shared_ptr<std::vector<std::string>> CFG::get_rand_production( const std::string& nt ) const {
    int index = rand() % ( rules.at(nt)->productions.size() );
    return rules.at(nt)->productions[index]; // Index into rule's productions vector
}

// Samples the given number of strings from the grammar, rejecting any longer than max_length
std::vector<std::string> CFG::sample_strings( const int& num, const int& max_length ) const {
    int samples_generated = 0;
        
    std::vector<std::string> results;
    while ( samples_generated < num ) {
        std::vector<std::string> output; // Holds terminals generated so far
        std::vector<std::string> output_stack;
        output_stack.push_back( start_symbol ); // We start with just S on the stack. Top is back
        
        // We carry out productions until the output stack has been emptied
        while ( output_stack.size() != 0 ) {
            if ( output.size() > max_length ) { // Reject long derivations as soon as they're too long
                break; // Exit output stack while loop
            }
            
            std::string token = output_stack.back();
            output_stack.pop_back(); // Pop the stack
            
            if ( in( token, terminals ) ) { // Token is a terminal => Add to output
                output.push_back( token );
                continue;                    
            } else if ( token.compare( "<epsilon>" ) == 0 ) { // Skip epsilon productions
                continue;
            } // Otherwise the token is a nonterminal
            
            std::shared_ptr<std::vector<std::string>> production = get_rand_production( token ); // Get random production
            for ( int i = production->size() - 1; i > -1; i-- ) {
                output_stack.push_back( production->at( i ) ); // Push tokens in reverse order
            }
        }
        
        if ( output.size() == 0 ) {
            std::cout << "Derived empty string..." << std::endl;
            continue; // Don't bother including empty strings in results
        } else if ( output.size() > max_length ) {
            std::cout << "Derived string longer than max_length..." << std::endl;
            continue; // Skip any derivations that are too long
        }
        
        // Otherwise construct and store the produced string
        std::string result_string = output[0];
        for ( int i = 1; i < output.size(); i++ ) {
            result_string += ( " " + output[i] );
        }
        results.push_back( result_string );
        samples_generated++;
    }
    return results;
}

std::vector<std::string> tokenize_on_spaces( const std::string& str ) {
    std::vector<std::string> tokens;
    
    std::size_t token_start = 0;
    std::size_t found = str.find_first_of(" "); // Position of first space in string
    while ( found != std::string::npos ) { // Indicates we passed the end of the string
        std::string current_token = str.substr( token_start, found - token_start );
        
        // Check if the current token is all spaces
        bool all_spaces = true;
        for ( int i = 0; i < current_token.length(); i++ ) {
            if ( current_token[i] != ' ' ) {
                all_spaces = false;
                break;
            }
        }
        
        if ( !all_spaces && ( current_token.length() > 0 ) ) {
            tokens.push_back( current_token );
        }
        
        token_start = found + 1;
        found = str.find_first_of(" ", found + 1); // Moves to next space
    }
    return tokens;
}

// Attempts to parse the given string into a Command object
std::shared_ptr<Command> CFG::parse( const std::string& str ) const {
    std::vector<std::string> tokens = tokenize_on_spaces( str );
    int current_token = 0;
    
    std::vector<std::string> stack; // Used to track what we expect to see next. Back = top
    // TODO - Finish this function and the project
}

std::ostream& operator<<( std::ostream& os, const CFG& cfg ) {
    os << "CFG{ Start symbol: " << cfg.start_symbol << std::endl;
    
    if ( cfg.nonterminals.size() != 0 ) { // Print nonterminals
        os << "\tNonterminals:" << std::endl;
        for ( int i = 0; i < cfg.nonterminals.size() - 1; i++ ) { // Skips last nt
            os << cfg.nonterminals[i] << " ";
        }
        os << cfg.nonterminals[cfg.nonterminals.size() - 1] << std::endl;
    } else {
        os << "Empty nonterminals" << std::endl;
    }
    
    if ( cfg.terminals.size() != 0 ) { // Print terminals
        os << "\tTerminals:" << std::endl;
        for ( int i = 0; i < cfg.terminals.size() - 1; i++ ) { // Skips last t
            os << cfg.terminals[i] << " ";
        }
        os << cfg.terminals[cfg.terminals.size() - 1] << std::endl;
    } else {
        os << "Empty terminals" << std::endl;
    }
    
    os << "TODO: Print rules if necessary }" << std::endl;
    
    return os;
}
