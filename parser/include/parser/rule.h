// Implements a Context-Free Grammar rule, i.e. a set of productions
// Author: Benned Hedegaard

#ifndef RULE_H
#define RULE_H

#include <string>
#include <vector>
#include <memory>

class Rule {
	public:
	
	    Rule( void ); // Empty constructor
		virtual ~Rule(); // Deconstructor
		Rule& operator=( const Rule& r ); // Assignment operator
		
		std::vector<std::shared_ptr<std::vector<std::string>>> productions;
};

#endif /* RULE_H */
