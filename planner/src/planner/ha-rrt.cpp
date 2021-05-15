// Implements the Homotopy-Aware RRT* planner introduced by Yi et al. (2016)
// Author: Benned Hedegaard

#include <chrono>
#include <iostream>
#include "planner/ha-rrt.h"

/*** Helper functions ***/

// Returns whether the line segment between (x1,y1) and (x2,y2) intersects the circle given by (xc,yc) and radius rc
bool intersects_circle( const double& x1, const double& y1, const double& x2, const double& y2, const double& xc, const double& yc, const double& rc ) {
    
    double a = ( x1 - x2 ) * ( x1 - x2 ) + ( y1 - y2 ) * ( y1 - y2 );
    double b = 2.0 * ( x1*x2 - x1*x1 + x1*xc - x2*xc + y1*y2 - y1*y1 + y1*yc - y2*yc );
    double c = x1*x1 + xc*xc + y1*y1 + yc*yc - rc*rc - 2.0*x1*xc - 2.0*y1*yc;
    
    double s = b*b - 4.0*a*c; // Negative s => No intersection
    
    if ( s < 0.0 ) {
        return false;
    }
    
    // Otherwise we have two possible points of intersection along the line
    double t1 = ( std::sqrt(s) - b ) / ( 2.0 * a );
    double t2 = ( - std::sqrt(s) - b ) / ( 2.0 * a );
    
    if ( t1 > 0.0 ) { // If either t1 or t2 is positive, the circle hits the line segment
        return true;
    }
    if ( t2 > 0.0 ) {
        return true;
    }
    
    return false;
}

// Returns a reversed copy of the given string
std::string reverse_copy( const std::string& s ) {
    std::string output = "";
    for ( int i = s.length() - 1; i > -1; i-- ) {
        output += s[i];
    }
    return output;
}

// Updates the best path stored for the homotopy class of ps in map P
void updateBestPath( const Path& p, std::shared_ptr<std::unordered_map<std::string, Path>>& P ) {
    int key_exists = P->count( p.str ); // Either 0 or 1
    if ( !key_exists ) { // Add this Path, as it's in a new homotopy class!
        P->insert({ p.str, p });
    } else if ( p.cost < P->at( p.str ).cost ) { // Homotopy class exists => is this Path better?
        P->at( p.str ) = p;
    }
}

// Removes any recursively embedded palindromic substrings from string v
std::string REPTrim( const std::string& v ) {
    std::vector<char> stack;
    for ( int i = 0; i < v.length(); i++ ) { // Iterate over characters in v
        if ( stack.size() == 0 ) { // Always push on empty stack
            stack.push_back( v[i] );
        } else if ( v[i] == stack.back() ) { // There's a character to compare; check if it's the same as v[i]
            stack.pop_back(); // Same characters cancel out, basically
        }
    }
    
    // Now turn into string and return
    std::string output = "";
    for ( int i = 0; i < stack.size(); i++ ) {
        output += stack[i];
    }
    return output;
}

// Merges paths in homotopy classes which are identical under REPTrim
std::unordered_map<std::string, Path> mergePaths( std::shared_ptr< std::unordered_map<std::string, Path> >& P ) {
    std::unordered_map<std::string, Path> output;
    for ( auto iter = P->begin(); iter != P->end(); ++iter ) {
        std::string rep_string = REPTrim( iter->first );
        Path p = iter->second;
        p.str = rep_string;
        
        int key_exists = output.count( rep_string ); // Either 0 or 1
        if ( !key_exists ) { // Add this Path, as it's in a new homotopy class!
            output.insert({ rep_string, p });
        } else if ( p.cost < output.at( rep_string ).cost ) { // Homotopy class exists => is this Path better?
            output.at( rep_string ) = p;
        }
    }
    return output;
}

// Samples a random number between the given doubles
double sample_random( const double& min, const double& max ) {
    double sample = ( rand() % 32760 ) / 32760.0; // Will between 0.0 and 1.0
    return ( min + sample * ( max - min ) );
}

/*** End helper functions ***/

// Constructor
HomotopyAwareRRT::HomotopyAwareRRT( const int& iterationsArg, const double& etaArg ) : num_iterations( iterationsArg ), eta( etaArg ), gamma( 1.0 ), has_world_model( false ) {

}

// Deconstructor
HomotopyAwareRRT::~HomotopyAwareRRT() {}

void HomotopyAwareRRT::handleWorldModel( const planner::WorldModelMsg::ConstPtr& msg ) {
	wm = WorldModel( *msg );
	has_world_model = true;
	
	std::cout << "World model has been received by HARRT*" << std::endl;
	//std::cout << wm << std::endl;
	
	// TODO - Move this to handleCommand instead. For now I choose a random obstacle: the first!
	frames = decomposeWorld( 0 );
	
	std::cout << "Reference frames have been computed and stored" << std::endl;
	publishFrames();
}

// Publishes the current stored reference frames
void HomotopyAwareRRT::publishFrames( void ) {
    planner::ReferenceFrameArrayMsg msg;
    
    for ( int i = 0; i < frames.size(); i++ ) {
        msg.frames.push_back( frames[i].to_msg() );
    }
    
    reference_frames_pub.publish( msg );
}

// Finds the reference frames for a particular obstacle of interest
std::vector<ReferenceFrame> HomotopyAwareRRT::decomposeWorld( const int& c_i ) const {
    
    // Seed a random number generator to sample points
    //std::chrono::system_clock::time_point time_now = std::chrono::high_resolution_clock::now();
    //unsigned seed = time_now.time_since_epoch().count();
    srand( 281 );
    
    // 1. Sample a point in each obstacle s.t. the point is not on a line connecting any other two representative points
    
    std::vector<Point> representatives;
    
    int i = 0; // Loops over wm.obstacles
    while ( i < wm.obstacles.size() ) {
    
        double obs_x = wm.obstacles[i].x;
        double obs_y = wm.obstacles[i].y;
        double obs_r = wm.obstacles[i].radius;
    
        // Sample point so that it's inside the obstacle
        Point h = Point( sample_random( obs_x - obs_r, obs_x + obs_r ), sample_random( obs_y - obs_r, obs_y + obs_r ), i );
        
        // Keep resampling until it's in the obstacle
        while ( obs_r < h.distance( obs_x, obs_y ) ) {
            h.x = sample_random( obs_x - obs_r, obs_x + obs_r );
            h.y = sample_random( obs_y - obs_r, obs_y + obs_r );
        }
        
        // Now check that the point does not fall between any other two representative points
        bool valid = true;
        for ( int j = 0; j < representatives.size(); j++ ) {
            for ( int k = 0; k < representatives.size(); k++ ) {
                if ( j == k ) continue;
                if ( h.same_line( representatives[j], representatives[k] ) ) {
                    valid = false;
                    break;
                }
            }
        }
        
        if ( valid ) { // This sampled representative point works, keep it and increment i
            representatives.push_back( h );
            i++;
        }
    }
    
    std::cout << "Sampled " << representatives.size() << " points for " << wm.obstacles.size() << " obstacles" << std::endl;
    
    // 2. Sample center point c within one of the obstacles, again not on any line connecting two representative points
    
    double c_obs_x = wm.obstacles[c_i].x;
    double c_obs_y = wm.obstacles[c_i].y;
    double c_obs_r = wm.obstacles[c_i].radius;
    
    // Sample point so that it's inside the obstacle    
    Point c = Point( sample_random( c_obs_x - c_obs_r, c_obs_x + c_obs_r ), sample_random( c_obs_y - c_obs_r, c_obs_y + c_obs_r ), c_i );
    
    while ( true ) {
        
        // Keep resampling until it's in the obstacle
        while ( c_obs_r < c.distance( c_obs_x, c_obs_y ) ) {
            c.x = sample_random( c_obs_x - c_obs_r, c_obs_x + c_obs_r );
            c.y = sample_random( c_obs_y - c_obs_r, c_obs_y + c_obs_r );
        }
        
        // Now check that the point does not fall between any other two representative points
        bool valid = true;
        for ( int j = 0; j < representatives.size(); j++ ) {
            for ( int k = 0; k < representatives.size(); k++ ) {
                if ( j == k ) continue;
                if ( c.same_line( representatives[j], representatives[k] ) ) {
                    valid = false;
                    break;
                }
            }
        }
            
        if ( valid ) { // Keep sampled center point
            break;
        }
    }
    
    std::cout << "Center point is at (" << c.x << "," << c.y << ")" << std::endl;
    
    // 3. Create radial structure of lines from c to all representative points
    
    std::vector<Line> lines;
    
    for ( Point& p : representatives ) {
        lines.push_back( Line( c, p ) );
    }
    
    // 4. Compute the line segments for each line based on the obstacles and map boundary
    std::vector<ReferenceFrame> rfs;
    
    for ( int i = 0; i < lines.size(); i++ ) {
        // First, I'll find the resulting points for each line from obstacles
        std::vector<Point> obstacle_points;
        
        for ( int o = 0; o < wm.obstacles.size(); o++ ) {
        
            // Each obstacle is a circle, which makes things easier
            std::vector<Point> o_points = lines[i].intersect_circle( wm.obstacles[o].x, wm.obstacles[o].y, wm.obstacles[o].radius, o );
            if ( o_points.size() > 0 ) {
                for ( Point& p : o_points ) {
                    obstacle_points.push_back( p );
                }
            }
        }
        
        std::cout << "\nFound these obstacle points for line " << i << ":" << std::endl;
        for ( const Point& p : obstacle_points ) {
            std::cout << p << std::endl;
        }
        
        // Next, consider the intersection point for each world border
        std::vector<Point> border_points;
        
        Line top_wall = Line( Point( wm.xy_min, wm.xy_max, -1 ), Point( wm.xy_max, wm.xy_max, -1 ) );
        Line right_wall = Line( Point( wm.xy_max, wm.xy_max, -1 ), Point( wm.xy_max, wm.xy_min, -1 ) );
        Line bottom_wall = Line( Point( wm.xy_max, wm.xy_min, -1 ), Point( wm.xy_min, wm.xy_min, -1 ) );
        Line left_wall = Line( Point( wm.xy_min, wm.xy_min, -1 ), Point( wm.xy_min, wm.xy_max, -1 ) );
        
        std::vector<Point> top_point = lines[i].intersect_line( top_wall ); // C++ version of an Option type
        if ( top_point.size() != 0 ) {
            bool bad = (top_point[0].x > ( wm.xy_max + 0.001 )) || (top_point[0].x < ( wm.xy_min - 0.001 ))
                || (top_point[0].y > ( wm.xy_max + 0.001 )) || (top_point[0].y < ( wm.xy_min - 0.001 ));
            if ( !bad ) {
                border_points.push_back( top_point[0] );
            }
        }
        std::vector<Point> right_point = lines[i].intersect_line( right_wall );
        if ( right_point.size() != 0 ) {
            bool bad = (right_point[0].x > ( wm.xy_max + 0.001 )) || (right_point[0].x < ( wm.xy_min - 0.001 ))
                || (right_point[0].y > ( wm.xy_max + 0.001 )) || (right_point[0].y < ( wm.xy_min - 0.001 ));
            if ( !bad ) {
                border_points.push_back( right_point[0] );
            }
        }
        std::vector<Point> bottom_point = lines[i].intersect_line( bottom_wall );
        if ( bottom_point.size() != 0 ) {
            bool bad = (bottom_point[0].x > ( wm.xy_max + 0.001 )) || (bottom_point[0].x < ( wm.xy_min - 0.001 ))
                || (bottom_point[0].y > ( wm.xy_max + 0.001 )) || (bottom_point[0].y < ( wm.xy_min - 0.001 ));
            if ( !bad ) {
                border_points.push_back( bottom_point[0] );
            }
        }
        std::vector<Point> left_point = lines[i].intersect_line( left_wall );
        if ( left_point.size() != 0 ) {
            bool bad = (left_point[0].x > ( wm.xy_max + 0.001 )) || (left_point[0].x < ( wm.xy_min - 0.001 ))
                || (left_point[0].y > ( wm.xy_max + 0.001 )) || (left_point[0].y < ( wm.xy_min - 0.001 ));
            if ( !bad ) {
                border_points.push_back( left_point[0] );
            }
        }
        
        std::cout << "Found these boundary points for line " << i << ":" << std::endl;
        for ( const Point& p : border_points ) {
            std::cout << p << std::endl;
        }
        
        // We can now combine the two lists and sort by proximity to the center point
        obstacle_points.insert( obstacle_points.end(), border_points.begin(), border_points.end() );
        
        std::sort( obstacle_points.begin(), obstacle_points.end(), compare_x ); // Gives ascending order in x
        
        std::cout << "Found " << obstacle_points.size() << " points for line " << i << ". Sorted order:" << std::endl;
        for ( int j = 0; j < obstacle_points.size(); j++ ) {
            std::cout << obstacle_points[j] << std::endl;
        }
        
        // Now create line segments with alternating consecutive pairs of points, starting with leftmost on wall
        std::cout << "Reference frames for line " << i << " are:" << std::endl;
        for ( int segment_i = 0; segment_i < obstacle_points.size() - 1; segment_i += 2 ) {
            char id = 65 + segment_i; // Maps to ASCII A...
            ReferenceFrame r = ReferenceFrame( id, obstacle_points[segment_i], obstacle_points[segment_i + 1] );
            std::cout << r << std::endl;
            rfs.push_back( r );
        }
    }
    
    std::cout << "\nOverall reference frames:" << std::endl;
    for ( int rf_i = 0; rf_i < rfs.size(); rf_i++ ) {
        std::cout << rfs[rf_i] << std::endl;
    }
    
    return rfs;
}

// Computes gamma (used in HARRT* search) based on the world model
void HomotopyAwareRRT::update_gamma( void ) {
    // Gamma needs to be at least ( 2 ( 1 + 1/d ) ) ^ ( 1/d ) * ( mu( Xfree ) / zeta_d ) ^ ( 1/d ) to be optimal
    // d = dimension of the configuration space = 2
    //      => ( 2 ( 1 + 1/d ) ) ^ ( 1/d ) = ( 2 ( 1 + 0.5 ) ) ^ ( 0.5 ) = sqrt( 3 )
    // zeta_d is the volume of the unit ball in d dimensions => zeta_2 = pi * 1^2 = pi
    // mu( Xfree ) is the Lebesgue measure (volume) of the obstacle-free space
    
    // First, compute mu_Xfree based on the world model size and obstacles
    double wm_size = wm.xy_max - wm.xy_min;
    double mu_Xfree = wm_size * wm_size;
    for ( const planner::ObstacleMsg& o : wm.obstacles ) {
        mu_Xfree -= ( M_PI * o.radius * o.radius ); // Subtract area of each obstacle
    }
    
    // TODO - Long into the future, consider if we need to scale this based on the world model size?
    gamma = std::sqrt( 3.0 ) * std::sqrt( mu_Xfree / M_PI ); // Finally update gamma
}

// Gamma needs to be at least ( 2 ( 1 + 1/d ) ) ^ ( 1/d ) * ( mu( Xfree ) / zeta_d ) ^ ( 1/d ) to be optimal
// d is the dimension of the configuration space, here it's 2
// zeta_d is the volume of the unit ball in d dimensions
// mu( Xfree ) is the Lebesgue measure (volume) of the obstacle-free space

//  Plans an optimal path from x_start to x_goal obeying the stored homotopy command
std::unordered_map<std::string, Path> HomotopyAwareRRT::search( const std::shared_ptr<Node>& x_start, const std::shared_ptr<Node>& x_goal ) {

    // 1. Reseed the random number generator
    std::chrono::system_clock::time_point time_now = std::chrono::high_resolution_clock::now();
    unsigned seed = time_now.time_since_epoch().count();
    srand( 281 );
    
    // 2. Begin actual search process

    // Implements the PathBlocks idea, i.e. the best Path in each homotopy class
    auto P = std::make_shared< std::unordered_map<std::string, Path> >();
    
    // Initialize Ts and Tg with the start and goal nodes
    std::vector<std::shared_ptr<Node>> Ts;
    x_start->cost = 0.0; // The root of both trees has cost 0
    x_start->str = ""; // Start with empty homotopy string
    Ts.push_back( x_start );
    
    std::vector<std::shared_ptr<Node>> Tg;
    x_goal->cost = 0.0; // The root of both trees has cost 0
    x_goal->str = ""; // Start with empty homotopy string
    Tg.push_back( x_goal );
    
    // Main search loop
    int i = 0;
    while ( i < num_iterations ) { // We limit search to this many iterations
        std::shared_ptr<Node> x_s_new = explore( Ts, i, true );
        std::shared_ptr<Node> x_g_new = explore( Tg, i, false );
        
        Path ps = connect( x_s_new, Tg, true ); // Build path connecting x_s_new to Tg
        Path pg = connect( x_g_new, Ts, false ); // Build path connecting x_g_new to Ts
        
        updateBestPath( ps, P ); // Update the best path stored for the homotopy class of ps
        updateBestPath( pg, P ); // Update the best path stored for the homotopy class of pg
        
        i++;
    }
    
    std::unordered_map<std::string, Path> P_pruned = mergePaths( P );
    return P_pruned;
}

// TODO - Probably compute each node string once and never again, then reuse that stored value

// Expands the given tree by attempting to connect a randomly sampled new node to the tree
// Similar to the explore used in RRT*
std::shared_ptr<Node> HomotopyAwareRRT::explore( std::vector<std::shared_ptr<Node>>& T, const int& i, const bool& forward ) {
    std::shared_ptr<Node> x_rand = sample( i ); // Samples a random Node in free space
    std::shared_ptr<Node> x_nearest = nearest( T, x_rand ); // Finds the nearest neighbor in the tree
    std::shared_ptr<Node> x_new = steer( x_nearest, x_rand ); // Steer from nearest node in tree toward x_rand
    
    if ( obstacle_free( x_nearest, x_new ) ) { // Check that the line from x_nearest to x_new is free
        if ( string_check( x_nearest->str + crf( x_nearest, x_new ), forward ) ) {
            std::shared_ptr<Node> x_min = std::make_shared<Node>( *x_nearest ); // Copy x_nearest
            double c_min = x_nearest->cost + c_line( x_nearest, x_new ); // Find initial minimum cost to x_new
            
            std::vector<std::shared_ptr<Node>> X_near = near( T, x_new ); // Find Nodes close to x_new in T
            for ( std::shared_ptr<Node>& x_near : X_near ) {
                if ( obstacle_free( x_new, x_near ) ) {
                    if ( string_check( x_near->str + crf( x_near, x_new ), forward ) ) {
                        if ( ( x_near->cost + c_line( x_near, x_new ) ) < c_min ) {
                            x_min = x_near;
                            c_min = x_near->cost + c_line( x_near, x_new ); // Store new minimum cost
                        }
                    }
                }
            }
            
            x_new->cost = x_min->cost + c_line( x_min, x_new ); // Update x_new's cost
            x_new->parent = x_min; // Add x_min as the parent of x_new
            x_new->str = x_min->str + crf( x_min, x_new ); // Store string leading to x_new
            T.push_back( x_new ); // Add x_new to the tree
            
            // Check if we should rewire any Nodes in X_near
            for ( std::shared_ptr<Node>& x_near : X_near ) {
                if ( ( *x_near ) == ( *x_min ) ) {
                    continue;
                }
                if ( obstacle_free( x_new, x_near ) ) {
                    if ( string_check( x_new->str + crf( x_new, x_near ), forward ) ) {
                        if ( ( x_new->cost + c_line( x_new, x_near ) ) < x_near->cost ) {
                            x_near->cost = x_new->cost + c_line( x_new, x_near );
                            x_near->parent = x_new; // Replace the parent of x_near with x_new
                            x_near->str = x_new->str + crf( x_new, x_near ); // Update string to x_near
                        }
                    }
                }
            }
        }
    }
    
    return x_new;
}

// Returns the path constructed by connecting Node x_new to tree T
Path HomotopyAwareRRT::connect( const std::shared_ptr<Node>& x_new, const std::vector<std::shared_ptr<Node>>& T, const bool& x_new_forward ) const {
    Path min_path( true ); // Minimum-cost path found through x_new
    min_path.cost = -1.0; // Mark uninitialized cost with -1
    
    std::vector<std::shared_ptr<Node>> X_near = near( T, x_new ); // Find Nodes close to x_new in T
    if ( X_near.size() == 0 ) {
        return min_path;
    } // Otherwise we have at least one neighbor x_near
    
    for ( std::shared_ptr<Node>& x_near : X_near ) {
        if ( !obstacle_free( x_new, x_near ) ) { // Path hits obstacle => Move to next x_near
            continue;
        }
    
        if ( x_new_forward ) { // Case where x_new is in Ts, x_near in Tg
            std::string path_string = x_new->str + crf( x_new, x_near ) + reverse_copy( x_near->str );
            if ( string_check( path_string, true ) ) { // String was constructed in forward direction
                if ( min_path.cost != -1.0 ) { // We should check to see if this path is better
                    if ( min_path.cost < ( x_new->cost + c_line( x_new, x_near ) + x_near->cost ) ) {
                        continue; // Go to next x_near
                    }
                } // If we make it here, store min_path
                
                min_path.nodes.clear(); // Reset min_path

                std::shared_ptr<Node> Ts_iterator = x_new; // We'll step backwards through Ts
                while ( Ts_iterator->parent != nullptr ) {
                    min_path.nodes.insert( min_path.nodes.begin(), Ts_iterator ); // Prepend to the path
                    Ts_iterator = Ts_iterator->parent;
                }
                min_path.nodes.insert( min_path.nodes.begin(), Ts_iterator ); // Add the root node too

                std::shared_ptr<Node> Tg_iterator = x_near; // We'll step forwards through Tg
                while ( Tg_iterator->parent != nullptr ) {
                    min_path.nodes.push_back( Tg_iterator ); // Append to the path
                    Tg_iterator = Tg_iterator->parent;
                }
                min_path.nodes.push_back( Tg_iterator ); // Add the root node too

                min_path.cost = x_new->cost + c_line( x_new, x_near ) + x_near->cost;
                min_path.str = path_string; // Store cost and homotopy string
            }
        } else { // Case where x_new is in Tg, x_near in Ts
            std::string path_string = x_near->str + crf( x_near, x_new ) + reverse_copy( x_new->str );
            if ( string_check( path_string, true ) ) { // String was constructed in forward direction
                if ( min_path.cost != -1.0 ) { // We should check to see if this path is better
                    if ( min_path.cost < ( x_near->cost + c_line( x_near, x_new ) + x_new->cost ) ) {
                        continue; // Go to next x_near
                    }
                } // If we make it here, store min_path
                
                min_path.nodes.clear(); // Reset min_path

                std::shared_ptr<Node> Ts_iterator = x_near; // We'll step backwards through Ts
                while ( Ts_iterator->parent != nullptr ) {
                    min_path.nodes.insert( min_path.nodes.begin(), Ts_iterator ); // Prepend to the path
                    Ts_iterator = Ts_iterator->parent;
                }
                min_path.nodes.insert( min_path.nodes.begin(), Ts_iterator ); // Add the root node too

                std::shared_ptr<Node> Tg_iterator = x_new; // We'll step forwards through Tg
                while ( Tg_iterator->parent != nullptr ) {
                    min_path.nodes.push_back( Tg_iterator ); // Append to the path
                    Tg_iterator = Tg_iterator->parent;
                }
                min_path.nodes.push_back( Tg_iterator ); // Add the root node too

                min_path.cost = x_near->cost + c_line( x_near, x_new ) + x_new->cost;
                min_path.str = path_string; // Store cost and homotopy string
            }
        }
    } // End x_near for loop
    
    return min_path;
}

/* TODO - These might be nice functions to clean up connect()
    Path - Returns the path from the root of the tree T to the vertex v.
    Input: Vertex v, Tree T
    Output: Path P

    Concat - Concatenates the two given paths.
    Input: Paths a and b
    Output: Concatenated path ab
*/

// Returns whether the given point falls inside an obstacle in the current world model
bool HomotopyAwareRRT::inside_obstacle( const double& x, const double& y ) const {
    for ( const planner::ObstacleMsg& o : wm.obstacles ) {
        if ( distance( x, y, o.x, o.y ) < o.radius ) {
            return true;
        }
    } // If we reach here, the point was inside no obstacle => false
    return false;
}

// Randomly samples a Node from obstacle-free configuration space
std::shared_ptr<Node> HomotopyAwareRRT::sample( const int& i ) const {
    // TODO - I'm unsure how we would use i, but this should work:
    double h_x = sample_random( wm.xy_min, wm.xy_max );
    double h_y = sample_random( wm.xy_min, wm.xy_max );

    // Resample until point falls outside an obstacle
    while ( inside_obstacle( h_x, h_y ) ) {
        h_x = sample_random( wm.xy_min, wm.xy_max );
        h_y = sample_random( wm.xy_min, wm.xy_max );
    }
    
    return std::make_shared<Node>( h_x, h_y ); // Build and return result
}

// Steers from the first Node toward the second Node a distance of eta
std::shared_ptr<Node> HomotopyAwareRRT::steer( const std::shared_ptr<Node>& n1, const std::shared_ptr<Node>& n2 ) const {
    double angle = atan2( n2->y - n1->y, n2->x - n1->x );
    return std::make_shared<Node>( n1->x + eta*cos( angle ), n1->y + eta*sin( angle ) );
}

// Returns the Nodes in the given tree within radius r of the given Node
std::vector<std::shared_ptr<Node>> HomotopyAwareRRT::near( const std::vector<std::shared_ptr<Node>>& T, const std::shared_ptr<Node>& n ) const {
    double r = std::min( gamma*std::log( T.size() ) / T.size(), eta ); // Radius prescribed by RRT* paper

    std::vector<std::shared_ptr<Node>> neighbors; // We'll output this
    for ( const std::shared_ptr<Node>& n_tree : T ) { // Iterate over all Nodes in the tree...
        if ( distance( n->x, n->y, n_tree->x, n_tree->y ) < r ) {
            neighbors.push_back( n_tree ); // Add neighbors that are within r of n
        }
    }
    return neighbors;
}

// Checks if the line segment between the two Nodes lies in obstacle-free space
bool HomotopyAwareRRT::obstacle_free( const std::shared_ptr<Node>& n1, const std::shared_ptr<Node>& n2 ) const {
    for ( const planner::ObstacleMsg& o : wm.obstacles ) {
        if ( intersects_circle( n1->x, n1->y, n2->x, n2->y, o.x, o.y, o.radius ) ) {
            return false;
        }
    }
    return true; // Hit no obstacles -> Obstacle-free line segment
}

bool sort_frames_hit( const std::pair<double,char>& x, const std::pair<double,char>& y ) {
    return (x.first < y.first);
}

// Returns the crossed reference frames on the line from Node n1 to Node n2
std::string HomotopyAwareRRT::crf( const std::shared_ptr<Node>& n1, const std::shared_ptr<Node>& n2 ) const {
    Line l = Line( Point( n1->x, n1->y ), Point( n2->x, n2->y ) );

    std::vector<std::pair<double,char>> reference_frames_hit; // Store reference frames hit plus distance to n1
    for ( const ReferenceFrame& rf : frames ) {
        std::vector<Point> intersection = rf.l.intersect_line( l ); // Intersects node line with reference frame
        if ( intersection.size() != 0 ) { // The lines intersect!
            std::pair<double,char> intersection_pair( l.p1.distance( intersection[0] ), rf.id );
            reference_frames_hit.push_back( intersection_pair );
        }
    }
    
    // Now sort the frames hit and construct a string
    std::sort( reference_frames_hit.begin(), reference_frames_hit.end(), sort_frames_hit );
    std::string output = "";
    for ( int i = 0; i < reference_frames_hit.size(); i++ ) {
        output += reference_frames_hit[i].second; // Append each reference frame id in order from n1
    }
    
    return output;
}

// Checks if v is a substring of the current homotopy command
// The forward argument allows us to handle the backward tree correctly
bool HomotopyAwareRRT::string_check( const std::string& v, const bool& forward ) const {
    // Recall that command_homotopy_classes contains all valid homotopy strings. We only need one that works!
    int len = v.length();
    std::string v_copy = v;
    if ( !forward ) { // Flip v so that we can compare to homotopy classes without flipping them
        v_copy = reverse_copy( v );
    }
    
    for ( const std::string& s : command_homotopy_classes ) {
        
        std::string substring = "";
        if ( forward ) {
            substring = s.substr( 0, len );
        } else {
            substring = s.substr( s.length() - len, len );
        }
        
        if ( substring.compare( 0, len, v_copy ) == 0 ) { // Strings are identical!
            return true;
        }
    } // Never had a match => Invalid substring
    return false;
}

// Returns the Euclidean distance between the given (x,y) points
double HomotopyAwareRRT::distance( const double& x1, const double& y1, const double& x2, const double& y2 ) const {
    return std::sqrt( std::pow( x1 - x2, 2.0 ) + std::pow( y1 - y2, 2.0 ) );
}

// Returns the cost of the line between the two nodes
double HomotopyAwareRRT::c_line( const std::shared_ptr<Node>& n1, const std::shared_ptr<Node>& n2 ) const {
    // TODO - Add other cost function options. For now, distance!
    return distance( n1->x, n1->y, n2->x, n2->y );
}

// Returns the closest Node to the given Node in the given tree
std::shared_ptr<Node> HomotopyAwareRRT::nearest( const std::vector<std::shared_ptr<Node>>& T, const std::shared_ptr<Node>& n ) const {
    int closest_node = 0; // Index
    double closest_distance = distance( T[0]->x, T[0]->y, n->x, n->y );
    
    for ( int i = 1; i < T.size(); i++ ) {
        double d = distance( T[i]->x, T[i]->y, n->x, n->y );
        if ( d < closest_distance ) {
            closest_node = i;
            closest_distance = d;
        }
    }
    return T[closest_node];
}
