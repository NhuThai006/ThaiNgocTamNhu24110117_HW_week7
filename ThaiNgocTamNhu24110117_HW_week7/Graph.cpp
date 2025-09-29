#include "Graph.h"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iostream>

void Graph::addEdge(const std::string& source, const std::string& dest, int weight) {
    adjacencyList[source].push_back({dest, weight});
    adjacencyList[dest].push_back({source, weight}); // Undirected graph
}

std::vector<std::string> Graph::dijkstra(const std::string& start, const std::string& end, int& totalDistance) {
    std::unordered_map<std::string, int> distances;
    std::unordered_map<std::string, std::string> previous;
    
    // Initialize distances
    for (const auto& vertex : adjacencyList) {
        distances[vertex.first] = std::numeric_limits<int>::max();
    }
    distances[start] = 0;

    // Priority queue for vertices
    std::priority_queue<std::pair<int, std::string>,
                       std::vector<std::pair<int, std::string>>,
                       std::greater<std::pair<int, std::string>>> pq;
    pq.push({0, start});

    while (!pq.empty()) {
        std::string current = pq.top().second;
        int currentDist = pq.top().first;
        pq.pop();

        if (currentDist > distances[current]) continue;

        for (const Edge& edge : adjacencyList[current]) {
            int newDist = distances[current] + edge.weight;
            if (newDist < distances[edge.dest]) {
                distances[edge.dest] = newDist;
                previous[edge.dest] = current;
                pq.push({newDist, edge.dest});
            }
        }
    }

    // Reconstruct path
    std::vector<std::string> path;
    if (distances[end] == std::numeric_limits<int>::max()) {
        totalDistance = -1;
        return path;
    }

    totalDistance = distances[end];
    std::string current = end;
    while (current != start) {
        path.push_back(current);
        current = previous[current];
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<std::string> Graph::bellmanFord(const std::string& start, const std::string& end, int& totalDistance) {
    std::vector<std::string> vertices = getVertices();
    std::unordered_map<std::string, int> distances;
    std::unordered_map<std::string, std::string> previous;
    
    // Initialize distances
    for (const auto& vertex : vertices) {
        distances[vertex] = std::numeric_limits<int>::max();
    }
    distances[start] = 0;

    // Relax edges |V|-1 times
    for (size_t i = 0; i < vertices.size() - 1; i++) {
        for (const auto& vertex : adjacencyList) {
            for (const Edge& edge : vertex.second) {
                if (distances[vertex.first] != std::numeric_limits<int>::max() &&
                    distances[vertex.first] + edge.weight < distances[edge.dest]) {
                    distances[edge.dest] = distances[vertex.first] + edge.weight;
                    previous[edge.dest] = vertex.first;
                }
            }
        }
    }

    // Check for negative cycles
    for (const auto& vertex : adjacencyList) {
        for (const Edge& edge : vertex.second) {
            if (distances[vertex.first] != std::numeric_limits<int>::max() &&
                distances[vertex.first] + edge.weight < distances[edge.dest]) {
                // Negative cycle detected
                totalDistance = -1;
                return std::vector<std::string>();
            }
        }
    }

    // Reconstruct path
    std::vector<std::string> path;
    if (distances[end] == std::numeric_limits<int>::max()) {
        totalDistance = -1;
        return path;
    }

    totalDistance = distances[end];
    std::string current = end;
    while (current != start) {
        path.push_back(current);
        current = previous[current];
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());
    return path;
}

void Graph::floydWarshall(std::vector<std::vector<int>>& distances, std::vector<std::vector<std::string>>& paths) {
    std::vector<std::string> vertices = getVertices();
    int V = vertices.size();
    
    // Initialize distances and paths matrices
    distances = std::vector<std::vector<int>>(V, std::vector<int>(V, std::numeric_limits<int>::max()));
    paths = std::vector<std::vector<std::string>>(V, std::vector<std::string>(V));

    // Initialize with direct edges
    for (int i = 0; i < V; i++) {
        distances[i][i] = 0;
        for (const Edge& edge : adjacencyList[vertices[i]]) {
            int j = std::find(vertices.begin(), vertices.end(), edge.dest) - vertices.begin();
            distances[i][j] = edge.weight;
            paths[i][j] = edge.dest;
        }
    }

    // Floyd-Warshall algorithm
    for (int k = 0; k < V; k++) {
        for (int i = 0; i < V; i++) {
            for (int j = 0; j < V; j++) {
                if (distances[i][k] != std::numeric_limits<int>::max() &&
                    distances[k][j] != std::numeric_limits<int>::max() &&
                    distances[i][k] + distances[k][j] < distances[i][j]) {
                    distances[i][j] = distances[i][k] + distances[k][j];
                    paths[i][j] = paths[i][k];
                }
            }
        }
    }
}

bool Graph::readFromFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) return false;

    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string source, dest;
        int weight;
        ss >> source >> dest >> weight;
        addEdge(source, dest, weight);
    }
    return true;
}

std::vector<std::string> Graph::getVertices() const {
    std::vector<std::string> vertices;
    for (const auto& vertex : adjacencyList) {
        vertices.push_back(vertex.first);
    }
    return vertices;
}