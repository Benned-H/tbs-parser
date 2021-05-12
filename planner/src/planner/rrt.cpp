// Implements the RRT* tree for the HARRT* planner
// Author: Benned Hedegaard

#include "planner/rrt.h"

// Empty constructor
RRT::RRT( void ) : nodes(), edges() {

}

// Deconstructor
RRT::~RRT() {}

// Expands the tree by randomly sampling a new node and attempting to connect it to the tree
Point RRT::explore( const int& i ) { /*
    Point x_rand = sample( i ); // TODO - Implement
    /*Point x_nearest = nearest( x_rand ); // TODO - Implement
    Point x_new = steer( x_nearest, x_rand, mu ); // TODO - Implement - where is mu?
    
    if ( obstacle_free( x_nearest, x_new ) ) { // TODO - Implement
        std::vector<char> s = str( x_nearest ); // TODO - Implement
        std::vector<char> s1 = crf( x_nearest, x_new ); // TODO - Implement
        s.insert( s.end(), s1.begin(), s1.end() ); // Concatenate the vectors
        
        if ( string_check( s ) ) { // TODO - Implement
            Point x_min = Point( x_nearest ); // Copy x_nearest
            std::vector<Point> X_near = near( x_new, |N| ); // TODO - Implement and where is N?
            
            for ( Point& x_near : X_near ) {
                if ( obstacle_free( x_new, x_near ) ) {
                    std::vector<char> s2 = str( x_near ); // TODO - Implement
                    std::vector<char> s3 = crf( x_near, x_new ); // TODO - Implement
                    s2.insert( s2.end(), s3.begin(), s3.end() ); // Concatenate the vectors
                    
                    if ( string_check( s2 ) ) {
                        if ( ( cost( x_near ) + c_line( x_near, x_new ) ) < cost( x_new ) ) {
                            x_min = x_near; // TODO - Implement Point assignment
                        }
                    }
                }
            }
            
            // TODO - Add ( x_min, x_new ) to E
            
            for ( Point& x_near : X_near ) {
                if ( x_near == x_min ) {
                    continue; // Skip x_min- TODO check correct
                }
                if ( obstacle_free( x_new, x_near ) ) { // TODO - Implement
                    std::vector<char> s2 = str( x_new ); // TODO - Implement
                    std::vector<char> s3 = crf( x_new, x_near ); // TODO - Implement
                    s2.insert( s2.end(), s3.begin(), s3.end() ); // Concatenate the vectors
                    
                    if ( string_check(s) ) {
                        if ( ( cost( x_new ) + c_line( x_new, x_near ) ) < cost( x_near ) ) {
                            Point x_parent = parent( x_near );
                            // TODO - Remove ( x_parent, x_near ) from edges
                            // TODO - Add ( x_new, x_near ) to edges
                        }
                    }
                }
            }
        }
    }
    
    return x_new;*/
}
