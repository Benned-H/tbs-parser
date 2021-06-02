// Implements a generic Context-Free Grammar
// Author: Benned Hedegaard

#include <stdlib.h>
#include <iostream>
#include "parser/cfg.h"

// Constructor
CFG::CFG( void ) : start_symbol( "" ), nonterminals(), terminals(), rules() {
    
}

// Deconstructor
CFG::~CFG() {}

// Reads and populates the CFG from a file in Backus-Naur Form
void CFG::from_file( const std::string& filename ) {
    std::ifstream myfile;
    myfile.open( filename ); // Open the file, which will default to input mode
    
    // Read each line and store them in a vector
    std::vector<std::string> lines;
    std::string line;
    if ( myfile.is_open() ) {
        while ( getline( myfile, line ) ) {
            std::cout << "Read the line: \"" << line << "\"" << std::endl;
            lines.push_back( line );
        }
    } else {
        std::cout << "Unable to read file \"" << filename << "\" in CFG::from_file()" << std::endl;
        return;
    }
    myfile.close();
    
    if ( lines.size() == 0 ) {
        std::cout << "File had no lines, exiting..." << std::endl;
        return;
    }
    
    // Now process the lines we read from file
    std::vector<std::string> tokens;
    std::string token;
    for ( int i = 0; i < lines.size(); i++ ) {
    
        // For each line, we first tokenize by spaces
        tokens = tokenize_on_spaces( lines[i] );
        std::cout << "Found " << tokens.size() << " tokens in the line: " << lines[i] << std::endl;
        
        // Now check that we have a valid production
        if ( tokens.size() < 3 ) {
            std::cout << "A production needs at least 3 tokens in the line, e.g. \"A := a\"" << std::endl;
            std::cout << lines[i] << std::endl;
            return;
        } else if ( tokens[1].compare("::=") != 0 ) {
            std::cout << "Rule did not have ::= symbol as second token, exiting..." << std::endl;
            std::cout << lines[i] << std::endl;
            return;
        }
        
        // Otherwise it's a presumably valid production
        if ( i == 0 ) { // Handle start symbol on first line of file
            start_symbol = tokens[0];
        }
        
        if ( !is_nonterminal( tokens[0] ) ) { // Handle any new nonterminals
            add_nonterminal( tokens[0] );
        }
        
        // Now segment out each production from the rule
        auto curr_production = std::make_shared<std::vector<std::string>>();
        for ( int t_i = 1; t_i < tokens.size(); t_i++ ) { // Loop over all other tokens
            if ( tokens[t_i].compare("|") == 0 ) { // Indicates that the current production is done
                if ( curr_production->size() <= 0 ) { // We had an empty production!
                    std::cout << "Production had no symbols, exiting..." << std::endl;
                    std::cout << lines[i] << std::endl;
                    return;
                } // Otherwise this rule is fine
                
                add_production( tokens[0], curr_production ); // Add curr_production to this nonterminal's rule
                curr_production = std::make_shared<std::vector<std::string>>(); // Reset curr_production
            } else {
                curr_production->push_back( tokens[t_i] );
            }
        }
        
        // Now that we've looped over all tokens, add one last production
        if ( curr_production->size() <= 0 ) { // We had an empty production!
            std::cout << "Production had no symbols, exiting..." << std::endl;
            std::cout << lines[i] << std::endl;
            return;
        } // Otherwise this rule is fine
        
        add_production( tokens[0], curr_production ); // Add curr_production to this nonterminal's rule
    }
}

// Adds a new production to the Rule for the given nonterminal
void CFG::add_production( const std::string& nt, const std::shared_ptr<std::vector<std::string>>& production ) {
    std::string token;
    for ( int i = 0; i < production->size(); i++ ) {
        token = production->at( i );
        
        // Handle new nonterminals
        if ( ( token.at( 0 ) == '<' ) && ( token.at( token.length() - 1 ) == '>' ) ) {
            if ( !is_nonterminal( token ) ) {
                add_nonterminal( token );
            }
        } else if ( !is_terminal( token ) ) { // Otherwise tokens are terminals
            add_terminal( token );
        }
    }
    
    rules[ nt ]->productions.push_back( production ); // Add production to this nonterminal's rule
}

// Adds a new terminal to the CFG
void CFG::add_terminal( const std::string& t ) {
    terminals.push_back( t );
}

// Returns whether the given string is already a terminal in the CFG
bool CFG::is_terminal( const std::string& t ) const {
    for ( const std::string& s : terminals ) {
        if ( s.compare( t ) == 0 ) {
            return true;
        }
    }
    return false;
}

// Adds a new nonterminal to the CFG
void CFG::add_nonterminal( const std::string& nt ) {
    nonterminals.push_back( nt );
    std::shared_ptr<Rule> empty_rule = std::make_shared<Rule>(); // Create empty rule for this nt
    rules.insert({ nt, empty_rule });
}

// Returns whether the given string is already a nonterminal in the CFG
bool CFG::is_nonterminal( const std::string& nt ) const {
    for ( const std::string& s : nonterminals ) {
        if ( s.compare( nt ) == 0 ) {
            return true;
        }
    }
    return false;
}

// Tokenizes the given string based on spaces
std::vector<std::string> CFG::tokenize_on_spaces( const std::string& str ) const {
    std::vector<std::string> tokens;
    std::string token;
    std::stringstream str_stream( str );
    
    while ( getline( str_stream, token, ' ' ) ) {
        tokens.push_back( token );
    }
    
    return tokens;
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
            
            if ( is_terminal( token ) ) { // Token is a terminal => Add to output
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
