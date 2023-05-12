#include "point.h"
#include "graph.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <cstdio>
#include <string>


int main(int argc, char* argv[]) {
    // check the argument 
    if (argc != 2) {
        std::cerr << "Error: invalid command" << std::endl;
        return EXIT_FAILURE;
    }

    std::ifstream input(argv[1]);
   
    // check if it is open
    if (!input.is_open()) {
        std::cerr << "Error: Cannot open file " << std::endl;
        return EXIT_FAILURE;
    }

    // read the nodes and edges by constructor
    Graph graph(input);

    // print the graph
    std::cout << graph << std::endl;

    return EXIT_SUCCESS;
}
