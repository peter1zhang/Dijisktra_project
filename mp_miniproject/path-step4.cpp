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
#include <stdexcept>

int main(int argc, char ** argv) {
    // check the number of command line arguments
    if (argc < 5 || argc > 6) {
        std::cerr << "Error: invalid command number" << std::endl;
        return EXIT_FAILURE;
    }

    // determine whether to use A* or Dijkstra's algorithm
    bool use_astar = false;
    if (argc == 6) {
        std::string option = argv[5];
        if (option == "-a") {
            use_astar = true;
        } else {
            std::cerr << "Error: invalid option " << std::endl;
            return EXIT_FAILURE;
        }
    }

    // read the grid map file and create the graph.
    std::ifstream input(argv[1]);
    if (!input.is_open()) {
        std::cerr << "Error: unable to open file " << std::endl;
        return EXIT_FAILURE;
    }

    // read the nodes and edges
    Graph graph(input);
    input.close();

    // read the obstacle file and update the graph
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

    // find the shortest path from the start node to the goal node
    unsigned int start_node = static_cast<unsigned int>(std::atoi(argv[3]));
    unsigned int goal_node = static_cast<unsigned int>(std::atoi(argv[4]));

    Path shortest_path;
    try {
        if (use_astar) {
            shortest_path = graph.astar(start_node, goal_node);
        } else {
            shortest_path = graph.dijkstra(start_node, goal_node);
        }
    } catch (const std::runtime_error& e) {
        std::cout << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    // print the shortest path
    std::cout << shortest_path << std::endl;

    return EXIT_SUCCESS;
}

