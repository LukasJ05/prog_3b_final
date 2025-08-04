#ifndef PATHFINDER_H
#define PATHFINDER_H

#include "Graph.h"
#include "Edge.h"
#include <vector>
#include <string>

class Pathfinder
{
private:
    const Graph& graph;

    // Heuristic function for A* (estimate of delay or distance)
    double heuristic(const std::string& from, const std::string& to) const;

public:
    Pathfinder(const Graph& g);

    // Returns the path as a vector of Edges
    std::vector<Edge> dijkstra(const std::string& start, const std::string& end);

    std::vector<Edge> astar(const std::string& start, const std::string& end);
};

#endif // PATHFINDER_H