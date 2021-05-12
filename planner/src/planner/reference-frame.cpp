// Implements a reference frame, which is a parameterized line segment and label
// Author: Benned Hedegaard

#include "planner/reference-frame.h"

// Constructor
ReferenceFrame::ReferenceFrame( const char& idArg, const Point& p1Arg, const Point& p2Arg ) : id( idArg ), l( p1Arg, p2Arg ) {

}

// Deconstructor
ReferenceFrame::~ReferenceFrame() {}

planner::ReferenceFrameMsg ReferenceFrame::to_msg( void ) const {
    planner::ReferenceFrameMsg msg;
    msg.p1 = l.p1.to_msg();
    msg.p2 = l.p2.to_msg();
    msg.id = std::string( 1, id );
    return msg;
}

std::ostream& operator<<( std::ostream& os, const ReferenceFrame& r ) {
    os << "ReferenceFrame{ l:" << r.l << ", id:" << r.id << " }";
    return os;
}
