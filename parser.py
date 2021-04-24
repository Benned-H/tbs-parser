# Parsing code to debug my TBS implementation
# By Benned Hedegaard

import random
#random.seed(281)

class CFG:
    def __init__(self):
        # Nonterminal start symbol of the CFG
        self.start_symbol = ''

        # List of nonterminal strings in the CFG, each of form "<nonterminal>"
        self.nonterminals = []

        # List of terminal strings in the CFG, each of form "terminal"
        self.terminals = []

        # Maps each nonterminal string to a list of valid rules
        # Each rule is a list of tokens, i.e. a list of nonterminals and/or terminals
        self.rules = {}

    def __repr__(self):
        output = "Start symbol: " + self.start_symbol + \
            "\n\nNonterminals: " + str(self.nonterminals) + \
            "\n\nTerminals: " + str(self.terminals) + "\n\nRules: "
        for key in sorted(self.rules):
            output = output + "\n\t" + key + " -> " + str(self.rules[key])
        return output

    # Sorts the terminal and nonterminal lists to make printouts clearer
    def sort_lists(self):
        self.terminals.sort()
        self.nonterminals.sort()

    # Adds a new nonterminal to the CFG
    def add_new_nonterminal(self, nt):
        self.nonterminals.append(nt)
        self.rules[nt] = []

    # Adds a new terminal to the CFG
    def add_new_terminal(self, t):
        self.terminals.append(t)

    # Adds a new production (list of tokens) for the given nonterminal to the CFG
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
        <nonterminal> ::= [<optional-term>] token_i ... token_j | token_k ... token_l | ...

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

                # We need at least 3 tokens for a rule, e.g. NT ::= token1
                if len(tokens) < 3:
                    print('Rule had less than three tokens, exiting...\n', line)
                    return
                elif tokens[1] != '::=':
                    print('Rule did not have ::= symbol as second token, exiting...\n', line)
                    return

                # Otherwise we assume the rule is valid and process accordingly
                if i == 0: # First line of file => Start symbol
                    self.start_symbol = tokens[0]
                if tokens[0] not in self.nonterminals:
                    self.add_new_nonterminal(tokens[0])

                curr_production = [] # We'll build up each production as we scan over the line
                for t in range(2, len(tokens)): # Loop over all tokens after ::=

                    if tokens[t] == '|': # Indicates that the current production is done
                        if len(curr_production) > 0:
                            self.add_new_production(tokens[0], curr_production)
                            curr_production = []
                        else:
                            print('Part of a rule had no symbols, exiting...\n', line)
                            return
                    else:
                        curr_production.append(tokens[t])

                # Having looped over all production tokens, add the final resulting production
                if len(curr_production) > 0:
                    self.add_new_production(tokens[0], curr_production)
                else:
                    print('Part of a rule had no symbols, exiting...\n', line)
                    return

        self.sort_lists()

    # Returns a random production for the given nonterminal
    def get_rand_production(self, nt, debug):
        options = self.rules[nt]
        if debug:
            print("Options are ", options)
        return random.choice(options)

    # Samples num random strings from the CFG
    # TODO - I currently do not handle optional items in the CFG such as [not], instead using <epsilon>
    def sample_strings(self, num = 10, debug = True):
        results = []
        for i in range(num):

            output_stack = [self.start_symbol]
            output = [] # Terminals only

            while len(output_stack) != 0:
                if debug:
                    print("\nCurrent stack is", output_stack, "\nCurrent string is", output)
                token = output_stack.pop(0)
                if debug:
                    print("Popped token", token)
                
                if token in self.terminals:
                    output.append(token)
                    continue
                elif token == '<epsilon>':
                    continue

                # Otherwise the token is a nonterminal
                production = self.get_rand_production(token, debug)
                if debug:
                    print("Production is ", str(production))

                # Loop over tokens in the production, putting onto output_stack
                for t in range( len(production)-1, -1, -1 ):
                    output_stack.insert(0, production[t])

            if len(output) == 0:
                if debug:
                    print('Derived empty string...')
                continue

            # We now combine the output string list into an actual string
            result_string = output[0]
            for s in output[1:]:
                result_string = result_string + ' ' + s
            results.append(result_string)
        return results

    # Returns only the terminals in the given production, without repeats
    def get_terminals(self, production):
        prod_terminals = []
        for token in production:
            if (token in self.terminals) and (token not in prod_terminals):
                prod_terminals.append(token)
        return prod_terminals

    """
    Converts the CFG to Chomsky Normal Form, in which all rules are one of the following forms:

        A ::= B C
        A ::= a
        S ::= <epsilon>
    """
    def convert_to_CNF(self):
        # First, ensure that the start symbol does not appear on the right-hand side of any productions
        # We introduce a new start symbol to trivially ensure this
        print("Step 1 - New start symbol")

        new_start_symbol = "<S0>"
        i = 1
        while new_start_symbol in self.nonterminals:
            new_start_symbol = "<S" + str(i) + ">"
            i += 1
        #print("Using the start symbol:", new_start_symbol)

        self.add_new_nonterminal(new_start_symbol)
        self.add_new_production(new_start_symbol, [self.start_symbol])
        self.start_symbol = new_start_symbol

        self.sort_lists()

        # Second, we eliminate rules with nonsolitary terminals, i.e. rules with form:
        #   A ::= B ... a ... C
        # For each such terminal, we introduce a new nonterminal Na and the rule:
        #   Na ::= a
        # Each place the terminal appears, we also replace a with Na
        print("Step 2 - Remove nonsolitary terminals")

        # 1. Find all nonsolitary terminals we'll need to replace
        nonsolitary_terminals = []
        for nt in self.nonterminals:
            for production in self.rules[nt]:
                if len(production) > 1: # More than 1 token => any terminals will be nonsolitary
                    prod_terminals = self.get_terminals(production)
                    for t in prod_terminals:
                        if t not in nonsolitary_terminals:
                            nonsolitary_terminals.append(t)

        # 2. Introduce new nonterminals Na for each of the nonsolitary terminals a
        # 3. Add new rules for these new nonterminals, e.g. Na ::= a
        new_nt_map = {} # Maps terminal to its replacement nonterminal
        for t in nonsolitary_terminals:
            new_nt = "<" + t + ">" # Produce a new nonterminal string
            i = 0
            
            while new_nt in self.nonterminals:
                new_nt = "<" + t + str(i) + ">"
                i += 1
            #print("Using the nonterminal:", new_nt)

            new_nt_map[t] = new_nt

            # Also add the new nonterminal and its rule:
            self.add_new_nonterminal(new_nt)
            self.add_new_production(new_nt, [t])

        # 4. Replace any nonsolitary instances of the tokens with their new nonterminals
        for nt in self.nonterminals:
            new_rules = [] # We'll replace the list of rules for the nonterminal with this
            for production in self.rules[nt]:
                if len(production) > 1: # More than 1 token => any terminals will be nonsolitary
                    new_production = []
                    for token in production:
                        if token in self.terminals:
                            new_production.append(new_nt_map[token]) # Append corresponding nonterminal
                        else:
                            new_production.append(token) # Append nonterminals as they appeared
                    new_rules.append(new_production)
                elif len(production) == 1:
                    new_rules.append(production) # 1 token => Just copy over production. Any with zero are deleted by omission
            self.rules[nt] = new_rules

        self.sort_lists()
        print(self)

        # Third, we eliminate right-hand side productions with more than 2 nonterminals, i.e. rules of the form:
        #   A ::= X1 X2 X3 ...
        # These rules are turned into:
        #   A ::= X1 A1
        #   A1 ::= X2 X3 ...
        print("Step 3 - Remove productions with more than 2 nonterminals")

        changes_made = True # Indicates changes were made inside the loop
        while changes_made:
            changes_made = False

            for nt in self.nonterminals:
                new_rules = [] # We'll replace the list of rules for the nonterminal with this
                
                for production in self.rules[nt]:

                    if len(production) > 2: # More than 2 tokens => multiple nonterminals at this point
                        new_nt = "<" + nt[1:-1] + ">" # Create new nonterminal child of nt
                        i = 0

                        while new_nt in self.nonterminals:
                            new_nt = "<" + nt[1:-1] + str(i) + ">"
                            i += 1
                        #print("Using the nonterminal:", new_nt)

                        # First, we want the rule with form: A ::= X1 A1
                        new_rules.append([production[0], new_nt])

                        # Second, introduce the new nonterminal and its corresponding rule
                        self.add_new_nonterminal(new_nt)
                        self.add_new_production(new_nt, production[1:])

                        changes_made = True

                    else:
                        new_rules.append(production) # 1 token => Just copy over production

                self.rules[nt] = new_rules

        self.sort_lists()
        print(self)

        # Fourth, we need to eliminate epsilon rules.
        # We determine the set of nullable nonterminals, i.e. those which can derive epsilon
        # Then replace all rules involving nullable nonterminals with rules both with and without that nonterminal
        print("Step 4 - Remove epsilon rules")

        # 1. Find all nullable nonterminals
        # 1a. Find nonterminals which directly produce epsilon
        nullables = ["<epsilon>"]
        for nt in self.nonterminals:
            for production in self.rules[nt]:
                if len(production) == 1:
                    if production[0] == "<epsilon>":
                        nullables.append(nt)

        # 1b. Find nonterminals which indirectly produce epsilon
        changes_made = True # Indicates changes were made inside the loop
        while changes_made:
            changes_made = False

            for nt in self.nonterminals:
                for production in self.rules[nt]:

                    # Check if all tokens in the production are known to be nullable
                    all_nullable = True
                    for token in production:
                        if token not in nullables:
                            all_nullable = False
                            break

                    if all_nullable and (nt not in nullables):
                        nullables.append(nt)
                        changes_made = True
                        break

        print("Found nullables:", nullables)

        # 2. Replace rules involving these nullables with copies of the rules with and without the nullable
        # 3. Only keep rules resulting in non-epsilon productions
        for nt in self.nonterminals:
            new_rules = [] # We'll replace the list of rules for the nonterminal with this
            
            for production in self.rules[nt]:
                production_copies = [[]] # Avoids trying to append to None object
                for token in production:
                    if token in nullables: # Create copy of the rule with and without this token
                        production_copies = [ (x + [token]) for x in production_copies ] + production_copies
                    else: # Just keep the token
                        production_copies = [ (x + [token]) for x in production_copies ]
                new_rules = new_rules + production_copies

                #print("Production copies are:", production_copies)
            
            # Now delete any nullable productions for this nonterminal
            new_rules_no_eps = []
            for rule in new_rules:
                rule_nullable = True
                for token in rule:
                    if token not in nullables:
                        rule_nullable = False
                        break
                
                # Keep only non-nullable production rules
                if not rule_nullable:
                    new_rules_no_eps.append(rule)

            self.rules[nt] = new_rules_no_eps

        self.sort_lists()
        print(self)

        # Fifth, and finally, we remove unit rules, i.e. those of form:
        #   A ::= B
        # For rules such as this, for each rule of form:
        #   B ::= X1 ... Xn
        # We add the rule:
        #   A ::= X1 ... Xn
        # We can also remove the nonterminal B from the CFG entirely, as it's replaced with A
        print("Step 5 - Remove unit rules")

        # 1. Find rules of this form, replace them one by one

        changes_made = True # Indicates changes were made inside the loop, namely deleting nonterminals
        while changes_made:
            changes_made = False

            for A in self.nonterminals:
                if changes_made: # Exits to the top of the while loop
                    break
                A_new_rules = []
                for A_production in self.rules[A]:
                    print("Handling production:", A, "::=", A_production)

                    if (len(A_production) == 1) and (A_production[0] in self.nonterminals):
                        B = A_production[0]
                        for B_production in self.rules[B]:
                            A_new_rules.append(B_production) # Add rules of form A ::= X1 ... Xn

                        # Now remove B from the CFG, as A has replaced it
                        self.nonterminals = [ x for x in self.nonterminals if (x != B)]
                        del self.rules[B]

                        # Now replace B in any existing productions with A
                        for nt in self.nonterminals:
                            new_rules = []
                            for rule in self.rules[nt]:
                                new_rules.append([ A if (x == B) else x for x in rule ])
                            self.rules[nt] = new_rules

                        # Update start symbol if necessary
                        if self.start_symbol == B:
                            self.start_symbol = A
                        
                        changes_made = True
                    else:
                        A_new_rules.append(A_production) # Add other A rules as they were

                self.rules[A] = A_new_rules

        self.sort_lists()
        print(self)

    """
    Checks if the CFG is currently in Chomsky Normal Form, in which all rules are one of the following forms:

        A ::= B C
        A ::= a
        S ::= <epsilon>

    This provides a sanity check on the above method's functioning properly. Returns True/False if valid CNF/not
    """
    def check_CNF(self):
        for nt in self.nonterminals:
            for rule in self.rules[nt]:

                if len(rule) == 0:
                    print("Error: Rule has length 0", rule)
                    return False
                elif len(rule) > 2:
                    print("Error: Rule has length", len(rule), rule)
                    return False
                # Otherwise the rule has length 1 or 2...

                # Length 1 must be a single terminal
                if (len(rule) == 1) and (rule[0] not in self.terminals):
                    print("Error: Rule has length 1 but is not a single terminal", nt, "::=", rule)
                    return False
                
                # Length 2 must be two nonterminals
                if len(rule) == 2:
                    if rule[0] not in self.nonterminals:
                        print("Error: Rule has length 2 but is not two nonterminals because of", rule[0], "\n", nt, "::=", rule)
                        return False
                    elif rule[1] not in self.nonterminals:
                        print("Error: Rule has length 2 but is not two nonterminals because of", rule[1], "\n", nt, "::=", rule)
                        return False
                
                # Now check that all tokens used are valid
                for token in rule:
                    if token == "<epsilon>" and (nt != self.start_symbol):
                        print("Error: Rule contained <epsilon> token", rule)
                        return False
                    if (token not in self.terminals) and (token not in self.nonterminals):
                        print("Error: Rule contained unrecognized token:", token, rule)
                        return False

        # Check that all nonterminals have at least one production
        for nt in self.nonterminals:
            if len(self.rules[nt]) == 0:
                print("Error: Nonterminal had zero productions", nt)

        # Check that all terminals and nonterminals are found at least once
        found_map = {} # Maps token -> True/False if it can be produced

        for nt in self.nonterminals:
            found_map[nt] = False

        for t in self.terminals:
            found_map[t] = False

        curr_nt = 0
        curr_rule = 0
        changes_made = True
        while changes_made:
            changes_made = False

            for nt in self.nonterminals:
                for rule in self.rules[nt]:
                    for token in rule:
                        if (not found_map[token]) and found_map[nt]:
                            found_map[token] = True
                            changes_made = True

        # After we've checked every rule and no changes were made, all should be selected:
        for nt in self.nonterminals:
            if not found_map[nt]:
                print("Error: Nonterminal", nt, "was not found in any right-hand rule")
                return False

        for t in self.terminals:
            if not found_map[t]:
                print("Error: Terminal", t, "was not found in any right-hand rule")
                return False

        # Otherwise we can finally return True.
        return True

def main():

    tbs_cfg = CFG()
    tbs_cfg.import_from_BNF('tbs.txt')
    print(tbs_cfg)
    
    # results = tbs_cfg.sample_strings(3, False)
    # for r in results:
    #     print(r)

    tbs_cfg.convert_to_CNF()
    print(tbs_cfg)

    if tbs_cfg.check_CNF():
        print("Valid CNF was created!")
    else:
        print("Invalid CNF was produced, investigate why :(")

if __name__ == '__main__':
    main()