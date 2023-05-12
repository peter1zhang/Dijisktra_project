#include "edge.h"

#include <iostream>
#include <cmath>

// implementation for the point orientation
int Edge::orientation(const Point & p1, const Point & p2, const Point & p3) const {
    // calculate z which determine Orientation of three points
    double z = (p2.getY() - p1.getY()) * (p3.getX() - p2.getX()) - 
                  (p2.getX() - p1.getX()) * (p3.getY() - p2.getY());

    //check if the orientation is clockwise, counter, collinear
    if (z > 0) {
        // clockwise
        return 1; 
    } else if (z < 0) {
        return -1; 
        // counterclockwise
    } else {
        return 0; 
        // collinear
    }

}

// implementation to check if three collinear points p1, p2, p3, when p1 lies on line p2p3
bool Edge::segment(const Point & p1, const Point & p2, const Point & p3) const {

    if (p2.getX() <= std::max(p1.getX(), p3.getX()) && p2.getX() >= std::min(p1.getX(), p3.getX()) &&
        p2.getY() <= std::max(p1.getY(), p3.getY()) && p2.getY() >= std::min(p1.getY(), p3.getY())) {

        return true;
    }

    return false;
}


// implementation of intersect method to check if two edges intersect
bool Edge::intersect(const Edge & otherEdge) const {
    //  fint the four orientations
    int o1 = orientation(point1, point2, otherEdge.point1);
    int o2 = orientation(point1, point2, otherEdge.point2);
    int o3 = orientation(otherEdge.point1, otherEdge.point2, point1);
    int o4 = orientation(otherEdge.point1, otherEdge.point2, point2);
    
    // general cases
    if (o1 != o2 && o3 != o4) {
        return true;
    }
    
    // collinear check for special cases
    if (o1 == 0 && segment(point1, otherEdge.point1, point2)) {
        return true;
    }
    if (o2 == 0 && segment(point1, otherEdge.point2, point2)) {
        return true;
    }
    if (o3 == 0 && segment(otherEdge.point1, point1, otherEdge.point2)) {
        return true;
    }
    if (o4 == 0 && segment(otherEdge.point1, point2, otherEdge.point2)) {
        return true;
    }

    return false;
}

