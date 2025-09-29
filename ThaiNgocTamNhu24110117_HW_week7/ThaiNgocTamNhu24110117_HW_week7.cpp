// ThaiNgocTamNhu24110117_HW_week7.cpp
//

#include <iostream>
#include "Graph.h"
#include <iomanip>

void printPath(const std::vector<std::string>& path, int distance) {
    if (path.empty()) {
        std::cout << "No path exists!" << std::endl;
        return;
    }
    
    for (size_t i = 0; i < path.size(); ++i) {
        std::cout << path[i];
        if (i < path.size() - 1) {
            std::cout << " -> ";
        }
    }
    std::cout << "\nTotal distance: " << distance << std::endl;
}

void printFloydWarshallResults(const std::vector<std::vector<int>>& distances, 
                             const std::vector<std::string>& vertices) {
    std::cout << "\nFloyd-Warshall All-Pairs Shortest Paths:\n";
    std::cout << std::setw(15) << "From\\To";
    for (const auto& vertex : vertices) {
        std::cout << std::setw(15) << vertex;
    }
    std::cout << "\n";

    for (size_t i = 0; i < vertices.size(); ++i) {
        std::cout << std::setw(15) << vertices[i];
        for (size_t j = 0; j < vertices.size(); ++j) {
            if (distances[i][j] == std::numeric_limits<int>::max())
                std::cout << std::setw(15) << "INF";
            else
                std::cout << std::setw(15) << distances[i][j];
        }
        std::cout << "\n";
    }
}

int main() {
    Graph graph;

    if (!graph.readFromFile("graph.txt")) {
        std::cout << "Error reading graph file!" << std::endl;
        return 1;
    }

    std::string start = "Dyer";
    std::string end = "Warm_Springs";
    int distance;

    std::cout << "\nDijkstra's Algorithm:\n";
    std::cout << "Shortest path from " << start << " to " << end << ":\n";
    auto dijkstraPath = graph.dijkstra(start, end, distance);
    printPath(dijkstraPath, distance);

    std::cout << "\nBellman-Ford Algorithm:\n";
    std::cout << "Shortest path from " << start << " to " << end << ":\n";
    auto bellmanPath = graph.bellmanFord(start, end, distance);
    printPath(bellmanPath, distance);

    std::vector<std::vector<int>> distances;
    std::vector<std::vector<std::string>> paths;
    graph.floydWarshall(distances, paths);
    printFloydWarshallResults(distances, graph.getVertices());

    return 0;
}
