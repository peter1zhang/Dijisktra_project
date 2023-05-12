#include "path.h"
#include "graph.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <cstdio>
#include <cctype>

int main(int argc, char ** argv) {
    // check the argument 
    if (argc != 4) {
        std::cerr << "Error: invalid command" << std::endl;
        return EXIT_FAILURE;
    }

    std::ifstream input(argv[1]);
 
    // check if it is open
    if (!input.is_open()) {
         std::cerr << "Error: Cannot open file " << std::endl;
        return EXIT_FAILURE;
    }
    
    // read the nodes and edges (same as path-step1.cpp)
    Graph graph(input);
    input.close();
    
    // Check if the input is not a character
    for (int i = 0; argv[2][i] != '\0'; i++) {
        if (!isdigit(argv[2][i])) {
            std::cerr << "Error: Start node must be a non-negative integer." << std::endl;
            return EXIT_FAILURE;
        }
    }

    for (int i = 0; argv[3][i] != '\0'; i++) {
        if (!isdigit(argv[3][i])) {
            std::cerr << "Error: Goal node must be a non-negative integer." << std::endl;
            return EXIT_FAILURE;
        }
    }

    // try to convert start and goal node ids from arguments
    unsigned int start_node = static_cast<unsigned int>(std::atoi(argv[2]));
    unsigned int goal_node = static_cast<unsigned int>(std::atoi(argv[3]));

    try {
        // find the shortest path using Dijkstra's algorithm in step 2
        Path shortest_path = graph.dijkstra(start_node, goal_node);

        // print the shortest path
        std::cout << shortest_path << std::endl;
    } catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
