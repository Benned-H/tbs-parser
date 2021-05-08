// Author: Benned Hedegaard

#include "gui/gui.h"

// Constructor with robot diameter parameter.
GUI::GUI( void ) : has_world_model( false ) {

}

GUI::~GUI() {} // Deconstructor

void GUI::handleWorldModel( const planner::WorldModelMsg::ConstPtr& msg ) {
	world_model = *msg;
	has_world_model = true;
	update();
}

std_msgs::ColorRGBA GUI::color( double r, double g, double b, double a ) const {
	std_msgs::ColorRGBA c;
	c.r = r;
	c.g = g;
	c.b = b;
	c.a = a;
	return c;
}

// TODO - If RViz doesn't reset things, could we only plot some items once?
void GUI::update( void ) const {
	if ( has_world_model ) drawWorldModel();
}

// Provide RGB in 0-255 range
void GUI::drawCylinder( const double& x, const double& y, const double& radius, double r, double g, double b, int id ) const {
	visualization_msgs::Marker c;
	c.header.frame_id = "gui";
	c.header.stamp = ros::Time::now();
	c.ns = "gui";
	c.id = id; // Unique id for this marker in the namespace
	c.type = visualization_msgs::Marker::CYLINDER;
	c.action = visualization_msgs::Marker::ADD;
	
	c.pose.position.x = x;
	c.pose.position.y = y;
	c.pose.position.z = 0.25; // Offset to make bottom at 0
	c.pose.orientation.x = 0.0;
    c.pose.orientation.y = 0.0;
    c.pose.orientation.z = 0.0;
    c.pose.orientation.w = 1.0;
	
	c.scale.x = 2.0 * radius;
	c.scale.y = 2.0 * radius;
	c.scale.z = 0.5;
	
	std::cout << "drawCylinder has rgb: " << r << "," << g << "," << b << std::endl;
	c.color.r = r / 255.0;
	c.color.g = g / 255.0;
	c.color.b = b / 255.0;
	c.color.a = 0.95;
	
	c.lifetime = ros::Duration(); // Never auto-deletes.
	
	marker_pub.publish(c); // Publish cylinder
}

void GUI::drawWorldModel( void ) const {
    
    // First draw world boundaries
    visualization_msgs::Marker line_strip;
    
    line_strip.header.frame_id = "gui";
	line_strip.header.stamp = ros::Time::now();
	line_strip.ns = "gui";
	line_strip.id = 1; // Unique id for this marker in the namespace
	line_strip.type = visualization_msgs::Marker::LINE_STRIP;
	line_strip.action = visualization_msgs::Marker::ADD;
	
	geometry_msgs::Point p1;
	p1.x = world_model.x_min.data;
	p1.y = world_model.y_min.data;
	p1.z = 0.0;
	
	geometry_msgs::Point p2;
	p2.x = world_model.x_max.data;
	p2.y = world_model.y_min.data;
	p2.z = 0.0;
	
	geometry_msgs::Point p3;
	p3.x = world_model.x_max.data;
	p3.y = world_model.y_max.data;
	p3.z = 0.0;
	
	geometry_msgs::Point p4;
	p4.x = world_model.x_min.data;
	p4.y = world_model.y_max.data;
	p4.z = 0.0;
	
	geometry_msgs::Point p5;
	p5.x = world_model.x_min.data;
	p5.y = world_model.y_min.data;
	p5.z = 0.0;
	
	line_strip.points.push_back( p1 );
	line_strip.points.push_back( p2 );
	line_strip.points.push_back( p3 );
	line_strip.points.push_back( p4 );
	line_strip.points.push_back( p5 );
	
	line_strip.scale.x = 0.05; // Scales the size of the lines
	line_strip.color = color(0.0,0.0,255.0,0.8);
	line_strip.lifetime = ros::Duration(); // Never auto-deletes.
	
	marker_pub.publish( line_strip ); // Publish world boundaries
	
	// Now draw the obstacles
	
    for ( int i = 0; i < world_model.obstacles.size(); i++ ) {
        double x = world_model.obstacles[i].x.data;
        double y = world_model.obstacles[i].y.data;
        double r = world_model.obstacles[i].radius.data;
        int id = 100 + i; // Use ids 100 and onwards for the obstacles
        
        std::cout << "Drawing obstacle type " << world_model.obstacles[i].type.data << " at (" << x << "," << y << ") with radius " << r << std::endl;
        
        std_msgs::ColorRGBA c = color( 255.0, 255.0, 255.0, 1.0 );
        
        switch ( world_model.obstacles[i].type.data ) {
            case 0: // Barrels are brown (99,80,10)
                drawCylinder( x, y, r, 99.0, 80.0, 10.0, id );
                std::cout << "Obstacle is a barrel" << std::endl;
                break;
            case 1: // Bushes are green (50,227,47)
                drawCylinder( x, y, r, 50.0, 227.0, 47.0, id );
                std::cout << "Obstacle is a bush" << std::endl;
                break;
            case 2: // Cones are orange (255,95,20)
                drawCylinder( x, y, r, 255.0, 95.0, 20.0, id );
                std::cout << "Obstacle is a cone" << std::endl;
                break;
            case 3: // Hydrants are red (240,31,31)
                drawCylinder( x, y, r, 240.0, 31.0, 31.0, id );
                std::cout << "Obstacle is a hydrant" << std::endl;
                break;
            default: // Error case, leave the obstacle white
                drawCylinder( x, y, r, 255.0, 255.0, 255.0, id );
                std::cout << "Obstacle has unknown type" << std::endl; 
                break;
        }
	}
}
