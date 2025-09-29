#pragma once
#include <vector>
#include <string>
#include <unordered_map>
#include <limits>
#include <queue>

class Graph {
private:
    struct Edge {
        std::string dest;
        int weight;
    };
    
    std::unordered_map<std::string, std::vector<Edge>> adjacencyList;

public:
    void addEdge(const std::string& source, const std::string& dest, int weight);
    std::vector<std::string> dijkstra(const std::string& start, const std::string& end, int& distance);
    std::vector<std::string> bellmanFord(const std::string& start, const std::string& end, int& distance);
    void floydWarshall(std::vector<std::vector<int>>& distances, std::vector<std::vector<std::string>>& paths);
    bool readFromFile(const std::string& filename);
    std::vector<std::string> getVertices() const;
};