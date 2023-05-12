#ifndef EDGE_H
#define EDGE_H

#include "point.h"

class Edge {
private:
   // an edge encapsulates two points
   Point point1;
   Point point2;

public:
   // constructor with initilize list
   Edge(const Point & p1, const Point & p2) : point1(p1), point2(p2) {};
   
   // method to check if two edges intersect 
   bool intersect(const Edge & otherEdge) const;

   //check if three collinear points p1, p2, p3, when p1 lies on line p2p3
   bool segment(const Point & p1, const Point & p2, const Point & p3) const; 
   
   // helper function and member function of Point or Edge that determines the orientation of three points
   int orientation(const Point & p1, const Point & p2, const Point & p3) const;

};

#endif