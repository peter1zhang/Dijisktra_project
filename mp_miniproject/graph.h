#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <map>
#include <iostream>
#include <ostream>
#include "point.h"
#include "path.h"
#include "edge.h"

class Graph {
private:
  // spatial information about the nodes, we store as a vector
  std::vector<Point> nodes;

  // an adjacency list with a vector of maps, stores the edge information
  std::vector<std::map<unsigned int, double> >  adjacency_list;

public:
   //default constructor
   Graph();

   // constructor for istream
   explicit Graph(std::istream& input);

   // addNode and addEdge functions
   void addNode(unsigned int node_id, double x, double y);
   void addEdge(unsigned int node_id1, unsigned int node_id2);

   // operator overload for << 
   friend std::ostream & operator<<(std::ostream & stream, const Graph & graph); 

   //step2 find the shortest path use the Dijkstra's algorithm from start to goal
   Path dijkstra(unsigned int start_node, unsigned int goal_node);

   //step 3 add the obstacle , which would construct an edge for each pair of nodes,
   // iterate all edges in the graph,for each edge, check if it intersects any of the obstacle's edges and remove the graph edge if so.
   void Obstacle(std::vector<unsigned int> & obstacle);
   

   //step4 A* A star algorithm
   Path astar(unsigned int start_node, unsigned int goal_node);
};

#endif