// Implements the world model for our planner
// Author: Benned Hedegaard

#include <random>
#include <iostream>
#include <chrono>
#include "planner/world-model.h"

// Empty constructor
WorldModel::WorldModel( void ) : obstacles() {}

// Constructor
WorldModel::WorldModel( const double& sizeArg, const double& radMinArg, const double& radMaxArg ) : xy_min( - sizeArg / 2.0 ), xy_max( sizeArg / 2.0 ), radius_min( radMinArg ), radius_max( radMaxArg ) {

}

// Constructor from message
WorldModel::WorldModel( const planner::WorldModelMsg& msg ) : xy_min( msg.xy_min ), xy_max( msg.xy_max ), obstacles() {
    for ( int i = 0; i < msg.obstacles.size(); i++ ) {
        obstacles.push_back( msg.obstacles[i] );
    }
}

// Deconstructor
WorldModel::~WorldModel() {}

// Samples the given number of obstacles and adds them to the world
void WorldModel::sampleObstacles( const int& num, const int& seed ) {

    std::default_random_engine generator( seed );
    std::uniform_real_distribution<double> xy_sampler( xy_min, xy_max );
    std::uniform_real_distribution<double> r_sampler( radius_min, radius_max );
    std::uniform_real_distribution<double> label_sampler( 0.0, 400.0 );
    
    // Now sample the correct number of obstacles
    int successes = 0;
    while ( successes < num ) {
    
        // Create hypothesis Obstacle information
        double h_x = xy_sampler( generator );
        double h_y = xy_sampler( generator );
        double h_r = r_sampler( generator );
        
        // Check if this Obstacle would be valid
        
        // 1. Check map boundaries
        if ( ( ( h_x + h_r ) > xy_max ) || ( ( h_x - h_r ) < xy_min ) ) {
            continue;
        } else if ( ( ( h_y + h_r ) > xy_max ) || ( ( h_y - h_r ) < xy_min ) ) {
            continue;
        }
        
        // 2. Check other obstacles
        bool bad = false;
        for ( planner::ObstacleMsg& o : obstacles ) {
            double distance = std::sqrt( ( o.x - h_x ) * ( o.x - h_x ) + ( o.y - h_y ) * ( o.y - h_y ) );
            if ( ( distance - o.radius - h_r ) < 0.0 ) { // Intersects with another obstacle, skip!
                bad = true;
                break;
            }
        }
        
        if ( bad ) continue; // Jumps to top of while loop to try again
        
        // Otherwise, this obstacle works. Find a label for it and add it to our world model!
        
        // TODO Redo - This is a hack to get rng integers, std::random int generator is weird the first time it's used
        double label_random_number = label_sampler( generator );
        uint32_t new_type = 0;
        if ( label_random_number > 300.0 ) {
            new_type = 3;
        } else if ( label_random_number > 200.0 ) {
            new_type = 2;
        } else if ( label_random_number > 100.0 ) {
            new_type = 1;
        } // Else new_type is left at 0
        
        std::cout << "Label sampler gave " << new_type << std::endl;
        
        uint32_t new_number = 0;
        while ( true ) {
        
            bool found = false;
            for ( planner::ObstacleMsg& o : obstacles ) {
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
        
        planner::ObstacleMsg new_obstacle;
        new_obstacle.x = h_x;
        new_obstacle.y = h_y;
        new_obstacle.radius = h_r;
        new_obstacle.type = new_type;
        new_obstacle.number = new_number;
        obstacles.push_back( new_obstacle );
        successes++;
    }
    
    std::cout << "Generated " << successes << " obstacles!" << std::endl;
}

// Returns a message representation of this object
planner::WorldModelMsg WorldModel::to_msg( void ) const {
    planner::WorldModelMsg wm;
    
    wm.xy_min = xy_min;
    wm.xy_max = xy_max;
    
    wm.obstacles = obstacles;
    
    return wm;
}

std::ostream& operator<<( std::ostream& os, const WorldModel& wm ) {
    os << "WorldModel: xy[" << wm.xy_min << "," << wm.xy_max << "]" << std::endl;
    os << "\tObstacles {";
    for ( const planner::ObstacleMsg& obs : wm.obstacles ) {
        os << obs;
    }
    os << "}";
    return os;
}
