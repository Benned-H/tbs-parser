// Implements the world model for our planner
// Author: Benned Hedegaard

#include <random>
#include <iostream>
#include "planner/world-model.h"

// Constructor
WorldModel::WorldModel( const double& width, const double& height, const double& radMin, const double& radMax ) : x_min( width / 2.0 ), x_max( width / 2.0 ), y_min( height / 2.0 ), y_max( height / 2.0 ), obstacle_types() {
    obstacle_types.push_back("barrel");
    obstacle_types.push_back("bush");
    obstacle_types.push_back("cone");
    obstacle_types.push_back("hydrant");
}

// Deconstructor
WorldModel::~WorldModel() {}

// Samples the given number of obstacles and adds them to the world
void WorldModel::sampleObstacles( const int& num ) {

    // Create a random number generators to sample obstacle location and radius
    std::default_random_engine generator;
    std::uniform_real_distribution<double> x_sampler( x_min, x_max );
    std::uniform_real_distribution<double> y_sampler( y_min, y_max );
    std::uniform_real_distribution<double> r_sampler( radius_min, radius_max );
    
    // Now sample the correct number of obstacles
    int successes = 0;
    while ( successes < num ) {
    
        // Create hypothesis Obstacle information
        double h_x = x_sampler( generator );
        double h_y = y_sampler( generator );
        double h_r = r_sampler( generator );
        
        // Check if this Obstacle would be valid
        
        // 1. Check map boundaries
        if ( ( ( h_x + h_r ) > x_max ) || ( ( h_x - h_r ) < x_min ) ) {
            continue;
        } else if ( ( ( h_y + h_r ) > y_max ) || ( ( h_y - h_r ) < y_min ) ) {
            continue;
        }
        
        // 2. Check other obstacles
        bool bad = false;
        for ( Obstacle& o : obstacles ) {
            double distance = std::sqrt( ( o.x - h_x ) * ( o.x - h_x ) + ( o.y - h_y ) * ( o.y - h_x ) );
            if ( ( distance - o.radius - h_r ) < 0.0 ) { // Intersects with another obstacle, skip!
                bad = true;
                break;
            }
        }
        
        if ( bad ) continue; // Jumps to top of while loop to try again
        
        // Otherwise, this Obstacle works. Find a label for it and add it to our world model!
        std::uniform_int_distribution<int> label_sampler( 0, obstacle_types.size() - 1 );
        int label_index = label_sampler( generator );
        
        std::string new_name = obstacle_types[label_index];
        int name_number = 0;
        while ( true ) {
        
            bool found_name = false;
            for ( Obstacle& o : obstacles ) {
                if ( new_name.compare( o.label ) == 0 ) { // Strings are equal
                    found_name = true;
                    break; // Exit for loop
                }
            }
            
            if ( !found_name ) {
                break; // Exit while loop
            }
            
            // Otherwise, we need a new name
            name_number++;
            new_name = obstacle_types[label_index] + std::to_string( name_number );
        }
        
        Obstacle new_obstacle( h_x, h_y, h_r, new_name );
        obstacles.push_back( new_obstacle );
        successes++;
    }
    
    std::cout << "Generated " << successes << " obstacles!" << std::endl;
    return;
}
