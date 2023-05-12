#ifndef POINT_H
#define POINT_H

#include <iostream>

class Point {
private:
  // index is the node_id
  unsigned int idx; 
  // we have x and y coordinates
  double x;
  double y;

public:
  // default constructor
  Point() : idx(0), x(0.0), y(0.0) {}

  // constructor with initilize list
  Point(unsigned int idx_, double x_, double y_): idx(idx_), x(x_), y(y_) {};
  
  //functions to get the value of x and y
  double getX() const { 
  return x;
  }

  double getY() const {
  return y; 
  }

  // function that calculate the distanceFrom
  double distanceFrom(const Point & rhs) const;
  
  // Overload the << operator for the Point class
  friend std::ostream & operator<<(std::ostream & stream, const Point & point);
};


#endif
