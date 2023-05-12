#include "graph.h"
#include "path.h"

#include <iostream>
#include <limits>
#include <queue>
#include <set>
#include <sstream>
#include <cstdlib>
#include <cstdio>
#include <cmath>

// addNode for the graph class
void Graph::addNode(unsigned int node_id, double x, double y) {
    // initialize the new node
    Point node(node_id, x, y);
    
    // make sure the vector (nodes) have enthough size
    if (nodes.size() <= node_id) {
        nodes.resize(node_id + 1);
    }

    // the index of the vector (nodes) is the node ID 
    nodes[node_id] = node;

    // let the adjacency_list vector has enough space for the new node
    if (adjacency_list.size() <= node_id) {
        adjacency_list.resize(node_id + 1);
    }

}

// implementation for addEdge
void Graph::addEdge(unsigned int node_id1, unsigned int node_id2) {
    // check if node_id1 and node_id2 is in the nodes
    if (node_id1 > nodes.size() || node_id2 > nodes.size()) {
        throw std::out_of_range("Error: node_id is out of range");
    }

    // use distanceFrom to calculate distance between two nodes
    double distance = nodes[node_id1].distanceFrom(nodes[node_id2]);

    // add an edge between these two nodes
    adjacency_list[node_id1][node_id2] = distance;
    adjacency_list[node_id2][node_id1] = distance;

}

// << operator overload
std::ostream & operator<<(std::ostream & stream, const Graph & graph) {
   // print the nodes by iterator
   std::vector<Point>::const_iterator it = graph.nodes.begin();
   while (it != graph.nodes.end()) {
      stream << *it << " ";
      it++;
   }
   stream << std::endl;

   // print the adjacency list
   for (size_t i = 0; i < graph.adjacency_list.size(); i++) {
      stream << i << ": ";
      // loop through all adjacency list index i contains 
      for (std::map<unsigned int, double>::const_iterator adj_it = graph.adjacency_list[i].begin();
            adj_it != graph.adjacency_list[i].end(); ++adj_it) {
            // print the node_id and weight 
            stream << adj_it->first << "," << adj_it->second << " ";
        }
    stream << std::endl;
   }

   return stream;
}

// default constructor
Graph::Graph() {
    // Default constructor, if needed
}

// constructor for istream to read the nodes and the edges information
Graph::Graph(std::istream& input) {
    std::string line;

    try {
        // read the nodes from input file
        if (std::getline(input, line) && line == "$nodes") {
            while (std::getline(input, line)) {
                if (line == "$edges") {
                    break;
                }

                unsigned int node_id;
                double x, y;
                std::istringstream node_data(line);
                node_data >> node_id >> x >> y;

                if (node_data.fail() || !node_data.eof()) {
                    throw std::runtime_error("Error: Invalid node data");
                }

                addNode(node_id, x, y);
            }
        } else {
            throw std::runtime_error("Error: Missing $nodes section");
        }

        // the edges information
        while (std::getline(input, line)) {
            unsigned int node_id1, node_id2;
            std::istringstream edge_data(line);
            edge_data >> node_id1 >> node_id2;

            if (edge_data.fail() || !edge_data.eof()) {
                throw std::runtime_error("Error: Invalid edge data");
            }

            addEdge(node_id1, node_id2);
        }
    } catch (const std::exception &e) {
        throw;
    }
}


// step2: find shortest path by using  Dijkstra's algorithm
// custom comparator for the priority queue, always return the lowest cost with lowest node id
struct PathComparator {
    bool operator()(const Path& a, const Path& b) const {
        if (a.getCost() == b.getCost()) {
            // compare entire paths
            return a.getPath() > b.getPath(); 
        }
        return a.getCost() > b.getCost();
    }
};

// using Dijkstra's algorithm to find the shortest path from start_node to goal_node
Path Graph::dijkstra(unsigned int start_node, unsigned int goal_node) {
    // first we check if start_node and goal_node are valid
    if (start_node >= nodes.size() || goal_node >= nodes.size()) {
        throw std::runtime_error("Invalid start node or goal node");
    }

    // we create a priority queue to store unexplored paths using the custom comparator
    std::priority_queue<Path, std::vector<Path>, PathComparator> paths_queue;

    // make an initial path with only the start_node and push it to the priority queue
    Path initial_path;
    initial_path.plusNode(start_node, 0.0);
    paths_queue.push(initial_path);

    // also create a set to store nodes that already been visited
    std::set<unsigned int> visited_nodes;

    // loop until the priority queue is empty
    while (!paths_queue.empty()) {
       
        // top() returns a reference to the top element with the smallest cost
        Path current_path = paths_queue.top();
        // pop() removes the top element with the smallest cost.
        paths_queue.pop();

        // get the last node in the current path
        unsigned int current_node = current_path.getLastNode();

        // if the last node is the goal_node, return the current path as the shortest path
        if (current_node == goal_node) {
            return current_path;
        }

        // current node has not been visited yet
        if (visited_nodes.find(current_node) == visited_nodes.end()) {
            
            // we let the current node marked as visited
            visited_nodes.insert(current_node);

            // derive the neighbors of the current node
            std::map<unsigned int, double> neighbors = adjacency_list[current_node];

            for (std::map<unsigned int, double>::iterator it = neighbors.begin(); it != neighbors.end(); it++) {
                
                unsigned int neighbor_node = it->first;
                double edge_cost = it->second;

                // if the neighbor_node has not been visited yet
                if (visited_nodes.find(neighbor_node) == visited_nodes.end()) {
                    // create a new path by extending the current_path with the neighbor_node
                    Path new_path = current_path;
                    new_path.plusNode(neighbor_node, edge_cost);

                    // add the new path to the priority queue
                    paths_queue.push(new_path);

                }
             }
         }
     }

    // if there is no path between the start_node and the goal_node
    throw std::runtime_error("No path exists between the start and goal nodes.");
}

//step 3 implementation of obstacle
/*- A member function of Graph that takes a vector of nodes representing an
    obstacle, which would construct an edge for each pair of nodes, iterate over all edges in the graph and for each edge,
     check if it intersects any of the obstacle's edges and remove the graph edge if so.
*/
void Graph::Obstacle(std::vector<unsigned int> & obstacle) {
    // make a vector of edges to store .
    std::vector<Edge> ob_edges;

    // construct an edge for each pair of nodes based on the input obstacle nodes.
    for (size_t i = 0; i < obstacle.size(); i++) {
          Point start = nodes[obstacle[i]];
          Point end;

        if (i == obstacle.size() - 1) {
            // if it's the last node, connect it to the first node.
            end = nodes[obstacle[0]];
        } else {
            // otherwise, connect it to the next node.
            end = nodes[obstacle[i + 1]];
        }
      ob_edges.push_back(Edge(start, end));
    }

    // iterate over all edges in the graph
    size_t num = 0;
    while (num < adjacency_list.size()) {

        std::map<unsigned int, double>::iterator it = adjacency_list[num].begin();
        while (it != adjacency_list[num].end()) {
            // create an edge object representing the current edge in the graph.
            Edge graph_edge(nodes[num], nodes[it->first]);
            bool intersects_obstacle = false;

            // check if the graph edge intersects any of the obstacle edges.
            size_t j = 0;
            while (j < ob_edges.size()) {
                if (graph_edge.intersect(ob_edges[j])) {
                    intersects_obstacle = true;
                    break;
                }
                j++;
            }

            // if the graph edge intersects an obstacle edge, remove it from the adjacency list.
            if (intersects_obstacle) {
                adjacency_list[num].erase(it++);
            } else {
                it++;
            }
        }

        num++;
    }
}


// step4 helper struct 
// Heuristic function using Euclidean distance calculates the estimated cost from a given node to the goal node
struct Heuristic {
    const std::vector<Point>& nodes; 
    unsigned int goal_node; 

    // takes a reference to the nodes vector and the goal node's index
    Heuristic(const std::vector<Point>& nodes_, unsigned int goal_node_)
    : nodes(nodes_), goal_node(goal_node_) {}

    // calculates the Euclidean distance between the given node and the goal node
    double operator()(unsigned int node) const {
        return nodes[node].distanceFrom(nodes[goal_node]);
    }
};

// comparison function for priority queue
struct Compare {
    Heuristic& heuristic; 

    Compare(Heuristic& heuristic_): heuristic(heuristic_) {}

    bool operator()(const Path &a, const Path &b) const {
        return a.getCost() + heuristic(a.getLastNode()) > b.getCost() + heuristic(b.getLastNode());
    }
};

// step4 A* A star implementation, use A* algorithm to find the shorest path 
Path Graph::astar(unsigned int start_node, unsigned int goal_node) {

    // initialize the costs, previous nodes, and visited nodes vectors
    std::vector<double> costs(nodes.size(), std::numeric_limits<double>::max());
    std::vector<unsigned int> previous(nodes.size(), static_cast<unsigned int>(-1));
    std::vector<bool> visited(nodes.size(), false);
    
    // ]heuristic function 
    Heuristic heuristic(nodes, goal_node);
    Compare compare(heuristic);
    std::priority_queue<Path, std::vector<Path>, Compare> pq(compare);
    
    // node cost 0.0
    costs[start_node] = 0.0;
    pq.push(Path(start_node, costs[start_node]));

    while (!pq.empty()) {
        Path current_path = pq.top();
        pq.pop();
        unsigned int current_node = current_path.getLastNode();

        // if the node is visited, continue with the next node
        if (visited[current_node]) {
            continue;
        }


        visited[current_node] = true;

        // if we reached the goal node, return the path
        if (current_node == goal_node) {
            return current_path;
        }

        // iterate through the neighbors of the current node
        for (std::map<unsigned int, double>::const_iterator it = adjacency_list[current_node].begin(); it != adjacency_list[current_node].end(); ++it) {
            unsigned int neighbor_node = it->first;
            double edge_cost = it->second;
            double new_cost = costs[current_node] + edge_cost;

            if (!visited[neighbor_node] && new_cost < costs[neighbor_node]) {
                costs[neighbor_node] = new_cost;
                previous[neighbor_node] = current_node;
                Path new_path = current_path;
                new_path.plusNode(neighbor_node, edge_cost);
                pq.push(new_path);
            }
        }
    }
     // No path found
    return Path();
}
