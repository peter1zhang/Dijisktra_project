#include "path.h"
#include "graph.h"
#include "point.h"
#include "edge.h"

#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <cstdio>

int main(int argc, char ** argv) {
    if (argc != 5) {
        std::cerr << "Error: invalid command number" << std::endl;
        return EXIT_FAILURE;
    }
    
    // read the grid map file and create the graph.
    std::ifstream input(argv[1]);
    if (!input.is_open()) {
        std::cerr << "Error: not open file" << std::endl;
        return EXIT_FAILURE;
    }
    
    // read the nodes and edges (same as path-step1.cpp)
    Graph graph(input);
    input.close();

    // read the obstacle file and update the graph.
    std::ifstream obstacle_file(argv[2]);
    if (!obstacle_file.is_open()) {
        std::cerr << "Error: Unable to open obstacle file" << std::endl;
        return EXIT_FAILURE;
    }

    std::string header;
    obstacle_file >> header;
    // check the obstacle files
    if (header == "$obstacles") {
        unsigned int node;
        std::vector<unsigned int> obstacle_nodes;
        while (obstacle_file >> node) {
            obstacle_nodes.push_back(node);
        }

         if (!obstacle_file.eof()) {
            obstacle_file.close();
            throw std::runtime_error("Error: Non-integer value found in obstacle nodes list.");
        }

        graph.Obstacle(obstacle_nodes);
    } else {
        obstacle_file.close();
        throw std::runtime_error("Error: '$obstacles' keyword not found in input file.");
    }

    obstacle_file.close();
    

    // check if the input is not a character
    for (int i = 0; argv[3][i] != '\0'; i++) {
        if (!isdigit(argv[3][i])) {
            std::cerr << "Error: Start node must be a non-negative integer." << std::endl;
            return EXIT_FAILURE;
        }
    }

    for (int i = 0; argv[4][i] != '\0'; i++) {
        if (!isdigit(argv[4][i])) {
            std::cerr << "Error: Goal node must be a non-negative integer." << std::endl;
            return EXIT_FAILURE;
        }
    }

    // try to convert start and goal node ids from arguments
    unsigned int start_node = static_cast<unsigned int>(std::atoi(argv[3]));
    unsigned int goal_node = static_cast<unsigned int>(std::atoi(argv[4]));
  
    try {
        // find the shortest path using Dijkstra's algorithm
        Path shortest_path = graph.dijkstra(start_node, goal_node);

        // print the shortest path
        std::cout << shortest_path << std::endl;
    } catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

