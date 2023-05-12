#ifndef PATH_H
#define PATH_H

#include <vector>
#include <iostream>

class Path {
private:
   // cost and ordered nodes for each path 
   double cost;
   std::vector<unsigned int> nodes;

public:
   //  default constructor with cost = 0.0 initilize
   Path(): cost(0.0) {}
   //Path(): cost(0.0) {}
   // add this constructor
    Path(unsigned int node, double cost_) : cost(cost_) {
        nodes.push_back(node);
    }

   // member functions: plusNode is add node, get Last node , get Cost
   void plusNode(unsigned int node, double cost);
   unsigned int getLastNode() const;
   double getCost() const;
   // get the list of nodes in the path
   const std::vector<unsigned int>& getPath() const {
        return nodes;
    }

   // operator> overload
   bool operator>(const Path &rhs) const;
   // operator<< overload 
   friend std::ostream & operator<<(std::ostream & stream, const Path & path);
   
};

#endif