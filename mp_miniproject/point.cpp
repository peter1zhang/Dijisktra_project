#include <iostream>
#include <cmath>
#include "point.h"

// implement the distance calculation between two nodes
double Point::distanceFrom(const Point & rhs) const {
    //calculate the distance between two nodes by using x and y
    double distance_x = x - rhs.x;
    double distance_y = y - rhs.y;

    // return a double which is the distance by using sqrt
    return std::sqrt(distance_x * distance_x + distance_y * distance_y);

}


// operator overload << to get the point x and y
std::ostream & operator<<(std::ostream & stream, const Point & point) {
    stream << "(" << point.x << "," << point.y << ")";
    return stream;
}
