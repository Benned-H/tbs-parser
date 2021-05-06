# Parsing code to debug my TBS implementation
# By Benned Hedegaard

import random
#random.seed(281)

class CFG:
    def __init__(self):
        self.start_symbol = ''
        self.nonterminals = [] # List of nonterminal strings
        self.terminals = [] # List of terminal strings
        self.rules = {} # Maps nonterminal to list of valid rules

    def __repr__(self):
        output = "Start symbol: " + self.start_symbol + \
            "\n\nNonterminals: " + str(self.nonterminals) + \
            "\n\nTerminals: " + str(self.terminals) + "\n\nRules: "
        for key in sorted(self.rules):
            output = output + "\n\t" + key + " -> " + str(self.rules[key])
        return output

    # Adds a new nonterminal to the CFG
    def add_new_nonterminal(self, nt):
        self.nonterminals.append(nt)
        self.rules[nt] = []

    # Adds a new terminal to the CFG
    def add_new_terminal(self, t):
        self.terminals.append(t)

    # Adds a new production for the given nonterminal
    def add_new_production(self, nt, production):
        for token in production:
            
            # Handle new nonterminals
            if token[0] == '<' and token[-1] == '>': # It's some nonterminal
                if token not in self.nonterminals: # And it's new!
                    self.add_new_nonterminal(token)

            # Handle new terminals
            elif token not in self.terminals:
                self.add_new_terminal(token)

        self.rules[nt].append(production)

    """
    Imports a CFG written in Backus-Naur Form from a file
    
    Assumes rule syntax:
        <nonterminal> ::= [<optional-term>] symbols ... | symbols ...

    The first line is assumed to be the start symbol. All other lines are assumed to be rules with
    all tokens separated by spaces. Do not put spaces within optional items as in [ <nonterminal> ]
    """
    def import_from_BNF(self, filename):
        with open(filename, 'r') as reader:
            lines = reader.readlines()
            if len(lines) == 0:
                print('File had no lines, exiting...')
                return

            # Otherwise we have lines to work with
            for i, line in enumerate(lines):
                
                # Tokenize by spaces and check that it's a valid rule
                tokens = line.strip().split(' ')
                if len(tokens) < 3:
                    print('Rule had less than three tokens, exiting...')
                    print(line)
                    return
                elif tokens[1] != '::=':
                    print('Rule did not have ::= symbol as second token, exiting...')
                    print(line)
                    return

                # Otherwise we assume the rule is valid, process accordingly
                if i == 0: # Handle start symbol
                    self.start_symbol = tokens[0]
                if tokens[0] not in self.nonterminals: # Add any new nonterminals
                    self.add_new_nonterminal(tokens[0])

                curr_production = []
                for t in range(2,len(tokens)): # Loop over all other tokens...
                    if tokens[t] == '|': # Indicates that the current production is done
                        if len(curr_production) > 0:
                            self.add_new_production(tokens[0], curr_production)
                            curr_production = []
                        else:
                            print('Part of a rule had no symbols, exiting...')
                            print(line)
                            return
                    else:
                        curr_production.append(tokens[t])
                # Having looped over all production tokens, add the resulting production
                if len(curr_production) > 0:
                    self.add_new_production(tokens[0], curr_production)
                else:
                    print('Part of a rule had no symbols, exiting...')
                    print(line)
                    return

    # Returns a random production for the given nonterminal
    def get_rand_production(self, nt):
        options = self.rules[nt]
        print("Options are ", options)
        return random.choice(options)

    def sample_strings(self, num=10):
        results = []
        for i in range(num):

            output_stack = [self.start_symbol]
            output = [] # Terminals only

            while len(output_stack) != 0:
                print()
                print("Current stack is", output_stack)
                print("Current string is", output)
                token = output_stack.pop(0)
                print("Popped token", token)
                
                if token in self.terminals:
                    output.append(token)
                    continue
                elif token == '<epsilon>':
                    continue

                # Otherwise the token is a nonterminal
                production = self.get_rand_production(token)
                print("Production is ", str(production))

                #output_stack.insert(0,']')
                for t in range(len(production)-1,-1,-1):
                    output_stack.insert(0,production[t])
                #output_stack.insert(0,'[')

            if len(output) == 0:
                print('Derived empty string...')
                continue

            result_string = output[0]
            for s in output[1:]:
                result_string = result_string + ' ' + s
            results.append(result_string)
        return results

def main():
    tbs_cfg = CFG()
    tbs_cfg.import_from_BNF('tbs.txt')
    print(tbs_cfg)
    
    results = tbs_cfg.sample_strings(10)
    for r in results:
        print(r)

if __name__ == '__main__':
    main()