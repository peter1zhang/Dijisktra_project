#include "path.h"
#include <iostream>

// add or plus (avoid name overwrite) a node to the path and update the cost
void Path::plusNode(unsigned int node, double new_cost) {
    nodes.push_back(node);
    // add to the cost with new cost
    cost += new_cost;
}

// we get the  last node in the path
unsigned int Path::getLastNode() const {
    return nodes.back();
}

// get the cost of the path
double Path::getCost() const {
    return cost;
}

// Overload the << operator f
std::ostream & operator<<(std::ostream & stream, const Path & path) {
    for (size_t i = 0; i < path.nodes.size(); ++i) {
        stream << path.nodes[i];
        if (i < path.nodes.size() - 1) {
            stream << " ";
        }
    }
    
    stream << " : " << path.cost;
    return stream;
}

// we also need to overload this to function normally
bool Path::operator>(const Path & rhs) const {
    return cost > rhs.cost;
}
