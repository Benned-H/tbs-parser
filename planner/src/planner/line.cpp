// Implements a line based on two points
// Author: Benned Hedegaard

#include <cmath>
#include "planner/line.h"

// Constructor
Line::Line( const Point& p1Arg, const Point& p2Arg ) : p1( p1Arg ), p2( p2Arg ) {

}

// Copy constructor
Line::Line( const Line& l ) : p1( l.p1 ), p2( l.p2 ) {
    
}

// Deconstructor
Line::~Line() {}

// Returns a vector containing the points at which this line intersects the given circle
std::vector<Point> Line::intersect_circle( const double& xc, const double& yc, const double& rc, const int& obstacle ) const {

    //std::cout << "Computing intersect_circle. This line has p1: " << p1 << " and p2: " << p2 << std::endl;
    
    double a = ( p1.x - p2.x ) * ( p1.x - p2.x ) + ( p1.y - p2.y ) * ( p1.y - p2.y );
    double b = 2.0 * ( p1.x*p2.x - p1.x*p1.x + p1.x*xc - p2.x*xc + p1.y*p2.y - p1.y*p1.y + p1.y*yc - p2.y*yc );
    double c = p1.x*p1.x + xc*xc + p1.y*p1.y + yc*yc - rc*rc - 2.0*p1.x*xc - 2.0*p1.y*yc;
    
    double s = b*b - 4.0*a*c; // Negative s => No intersection
    
    std::vector<Point> result;
    if ( s < 0.0 ) {
        return result;
    }
    
    // Otherwise we have two Points of intersection
    double t1 = ( std::sqrt(s) - b ) / ( 2.0 * a );
    double t2 = ( - std::sqrt(s) - b ) / ( 2.0 * a );
    
    double xt1 = p1.x + t1 * ( p2.x - p1.x );
    double yt1 = p1.y + t1 * ( p2.y - p1.y );
    double xt2 = p1.x + t2 * ( p2.x - p1.x );
    double yt2 = p1.y + t2 * ( p2.y - p1.y );
    
    result.push_back( Point( xt1, yt1, obstacle ) );
    result.push_back( Point( xt2, yt2, obstacle ) );
    
    //std::cout << "Exiting intersect_circle. Returning r1: " << result[0] << " and r2: " << result[1] << std::endl;
    
    return result;
}

// Returns a vector containing point at which this line intersects the given line
std::vector<Point> Line::intersect_line( const Line& l ) const {
    
    //std::cout << "Computing intersect_line. This line has p1: " << p1 << " and p2: " << p2 << std::endl;
    
    std::vector<Point> result;
    if ( l.p1.x == l.p2.x ) { // Other line is vertical!
        if ( p1.x == p2.x ) { // I'm vertical too!
            return result; // No point of intersection
        } else { // We intersect the vertical line at its x
            double m1 = ( p2.y - p1.y ) / ( p2.x - p1.x );
            
            double result_x = l.p1.x;
            double result_y = m1*result_x - m1*p1.x + p1.y;
            
            result.push_back( Point( result_x, result_y, -1 ) );
            return result;
        }
    } // Otherwise other line is not vertical
    
    if ( p1.x == p2.x ) { // I'm vertical though
        double m2 = ( l.p2.y - l.p1.y ) / ( l.p2.x - l.p1.x );
        
        // We intersect at my x
        double result_x = p1.x;
        double result_y = m2*result_x - m2*l.p1.x + l.p1.y;
        
        result.push_back( Point( result_x, result_y, -1 ) );
        return result;
    }
    
    // Otherwise no lines are vertical
    double m1 = ( p2.y - p1.y ) / ( p2.x - p1.x );
    double m2 = ( l.p2.y - l.p1.y ) / ( l.p2.x - l.p1.x );
    
    if ( m1 == m2 ) { // Lines cannot overlap => Parallel means no intersection
        return result;
    }
    
    double result_x = ( m1*p1.x + l.p1.y - m2*l.p1.x - p1.y ) / ( m1 - m2 );
    double result_y = m1*result_x - m1*p1.x + p1.y;
    result.push_back( Point( result_x, result_y, -1 ) );
    
    return result;
}

std::ostream& operator<<( std::ostream& os, const Line& l ) {
    os << "Line{ p1:" << l.p1 << ", p2:" << l.p2 << " }";
    return os;
}
