// Implements the world model for our planner
// Author: Benned Hedegaard

#include <random>
#include <iostream>
#include <chrono>
#include "planner/world-model.h"

// Constructor
WorldModel::WorldModel( const double& width, const double& height, const double& radMin, const double& radMax ) : x_min( - width / 2.0 ), x_max( width / 2.0 ), y_min( - height / 2.0 ), y_max( height / 2.0 ), radius_min( radMin ), radius_max( radMax ) {

}

// Deconstructor
WorldModel::~WorldModel() {}

// Samples the given number of obstacles and adds them to the world
void WorldModel::sampleObstacles( const int& num ) {

    // Create a random number generators to sample obstacle location and radius
    std::chrono::system_clock::time_point time_now = std::chrono::high_resolution_clock::now();
    unsigned seed = time_now.time_since_epoch().count();
    std::default_random_engine generator( seed );
    std::uniform_real_distribution<double> x_sampler( x_min, x_max );
    std::uniform_real_distribution<double> y_sampler( y_min, y_max );
    std::uniform_real_distribution<double> r_sampler( radius_min, radius_max );
    std::uniform_real_distribution<double> label_sampler( 0.0, 400.0 );
    
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
            double distance = std::sqrt( ( o.x - h_x ) * ( o.x - h_x ) + ( o.y - h_y ) * ( o.y - h_y ) );
            if ( ( distance - o.radius - h_r ) < 0.0 ) { // Intersects with another obstacle, skip!
                bad = true;
                break;
            }
        }
        
        if ( bad ) continue; // Jumps to top of while loop to try again
        
        // Otherwise, this Obstacle works. Find a label for it and add it to our world model!
        
        // TODO Redo - This is a hack to get rng integers, std::random int generator is weird the first time it's used
        double label_random_number = label_sampler( generator );
        uint32_t label_index = 0;
        if ( label_random_number > 300.0 ) {
            label_index = 3;
        } else if ( label_random_number > 200.0 ) {
            label_index = 2;
        } else if ( label_random_number > 100.0 ) {
            label_index = 1;
        } // Else leave it at 0
        
        std::cout << "Label sampler gave " << label_index << std::endl;
        
        ObstacleType new_type = ObstacleType::BARREL;
        
        if ( label_index == 1 ) {
            new_type = ObstacleType::BUSH;
        } else if ( label_index == 2 ) {
            new_type = ObstacleType::CONE;
        } else if ( label_index == 3 ) {
            new_type = ObstacleType::HYDRANT;
        }
        
        uint32_t new_number = 0;
        while ( true ) {
        
            bool found = false;
            for ( Obstacle& o : obstacles ) {
                if ( ( o.type == new_type ) && ( o.number == new_number ) ) { // Obstacles have same identifier
                    found = true;
                    break; // Exit for loop
                }
            }
            
            if ( !found ) {
                break; // Exit while loop
            }
            
            // Otherwise, we need a new number
            new_number++;
        }
        
        std::cout << "Obstacle is a " << new_type << " at (" << h_x << "," << h_y << ") with radius " << h_r << std::endl;
        Obstacle new_obstacle( h_x, h_y, h_r, new_type, new_number );
        obstacles.push_back( new_obstacle );
        successes++;
    }
    
    std::cout << "Generated " << successes << " obstacles!" << std::endl;
}

// Returns a message representation of this object
planner::WorldModelMsg WorldModel::to_msg( void ) const {
    planner::WorldModelMsg wm;
    
    wm.x_min.data = x_min;
    wm.x_max.data = x_max;
    wm.y_min.data = y_min;
    wm.y_max.data = y_max;
    
    for ( int i = 0; i < obstacles.size(); i++ ) {
        wm.obstacles.push_back( obstacles[i].to_msg() );
    }
    
    return wm;
}
