// Implements the Homotopy-Aware RRT* planner introduced by Yi et al. (2016)
// Author: Benned Hedegaard

#include <random>
#include <chrono>
#include <iostream>
#include "planner/ha-rrt.h"

// Constructor
HomotopyAwareRRT::HomotopyAwareRRT( const int& iterationsArg ) : num_iterations( iterationsArg ), has_world_model( false ) {

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
    
    // Create a random number generator to sample points
    //std::chrono::system_clock::time_point time_now = std::chrono::high_resolution_clock::now();
    //unsigned seed = time_now.time_since_epoch().count();
    std::default_random_engine generator( 281 );
    
    // 1. Sample a point in each obstacle s.t. the point is not on a line connecting any other two representative points
    
    std::vector<Point> representatives;
    
    int i = 0; // Loops over wm.obstacles
    while ( i < wm.obstacles.size() ) {
    
        double obs_x = wm.obstacles[i].x;
        double obs_y = wm.obstacles[i].y;
        double obs_r = wm.obstacles[i].radius;
    
        // Sample point so that it's inside the obstacle
        std::uniform_real_distribution<double> x_sampler( obs_x - obs_r, obs_x + obs_r );
        std::uniform_real_distribution<double> y_sampler( obs_y - obs_r, obs_y + obs_r );
        
        Point h = Point( x_sampler( generator ), y_sampler( generator ), i );
        
        // Keep resampling until it's in the obstacle
        while ( obs_r < h.distance( obs_x, obs_y ) ) {
            h.x = x_sampler( generator );
            h.y = y_sampler( generator );
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
    std::uniform_real_distribution<double> cx_sampler( c_obs_x - c_obs_r, c_obs_x + c_obs_r );
    std::uniform_real_distribution<double> cy_sampler( c_obs_y - c_obs_r, c_obs_y + c_obs_r );
    
    Point c = Point( cx_sampler( generator ), cy_sampler( generator ), c_i );
    
    while ( true ) {
        
        // Keep resampling until it's in the obstacle
        while ( c_obs_r < c.distance( c_obs_x, c_obs_y ) ) {
            c.x = cx_sampler( generator );
            c.y = cy_sampler( generator );
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
        
        Line top_wall = Line( Point( wm.x_min, wm.y_max, -1 ), Point( wm.x_max, wm.y_max, -1 ) );
        Line right_wall = Line( Point( wm.x_max, wm.y_max, -1 ), Point( wm.x_max, wm.y_min, -1 ) );
        Line bottom_wall = Line( Point( wm.x_max, wm.y_min, -1 ), Point( wm.x_min, wm.y_min, -1 ) );
        Line left_wall = Line( Point( wm.x_min, wm.y_min, -1 ), Point( wm.x_min, wm.y_max, -1 ) );
        
        std::vector<Point> top_point = lines[i].intersect_line( top_wall ); // C++ version of an Option type
        if ( top_point.size() != 0 ) {
            bool bad = (top_point[0].x > ( wm.x_max + 0.001 )) || (top_point[0].x < ( wm.x_min - 0.001 ))
                || (top_point[0].y > ( wm.y_max + 0.001 )) || (top_point[0].y < ( wm.y_min - 0.001 ));
            if ( !bad ) {
                border_points.push_back( top_point[0] );
            }
        }
        std::vector<Point> right_point = lines[i].intersect_line( right_wall );
        if ( right_point.size() != 0 ) {
            bool bad = (right_point[0].x > ( wm.x_max + 0.001 )) || (right_point[0].x < ( wm.x_min - 0.001 ))
                || (right_point[0].y > ( wm.y_max + 0.001 )) || (right_point[0].y < ( wm.y_min - 0.001 ));
            if ( !bad ) {
                border_points.push_back( right_point[0] );
            }
        }
        std::vector<Point> bottom_point = lines[i].intersect_line( bottom_wall );
        if ( bottom_point.size() != 0 ) {
            bool bad = (bottom_point[0].x > ( wm.x_max + 0.001 )) || (bottom_point[0].x < ( wm.x_min - 0.001 ))
                || (bottom_point[0].y > ( wm.y_max + 0.001 )) || (bottom_point[0].y < ( wm.y_min - 0.001 ));
            if ( !bad ) {
                border_points.push_back( bottom_point[0] );
            }
        }
        std::vector<Point> left_point = lines[i].intersect_line( left_wall );
        if ( left_point.size() != 0 ) {
            bool bad = (left_point[0].x > ( wm.x_max + 0.001 )) || (left_point[0].x < ( wm.x_min - 0.001 ))
                || (left_point[0].y > ( wm.y_max + 0.001 )) || (left_point[0].y < ( wm.y_min - 0.001 ));
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

//  Plans an optimal path from x_start to x_goal obeying a given homotopy specification
std::vector<Path> HomotopyAwareRRT::search( const Point& x_start, const Point& x_goal, const Action& a ) {/*
    int i = 0;
    
    // Initialize Ts and Tg with the start and goal nodes
    RRT Ts = RRT();
    Ts.nodes.push_back( x_start );
    RRT Tg = RRT();
    Tg.nodes.push_back( x_goal );
    
    while ( i < NUM_ITERATIONS ) {
        Point x_s_new = Ts.explore( i ); // TODO - Implement explore in RRT
        Point x_g_new = Tg.explore( i );
        
        Point ps = Tg.connect( x_s_new ); // TODO - Implement connect in RRT
        Point pg = Ts.connect( x_g_new );
        
        // TODO - Then update best paths in class
        P = UpdateBestPathByClass( ps, P )
        P = UpdateBestPathByClass( pg, P )
        
        i++;
    }
    
    // TODO then return MergePaths( P )*/
}
