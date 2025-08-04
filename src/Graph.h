#ifndef GRAPH_H
#define GRAPH_H

#include "Edge.h"
#include <unordered_map>
#include <vector>
#include <string>
using namespace std;

class Graph
{
private:
    unordered_map<string, vector<Edge>> adj;

public:
    void addEdge(const string& from, const string& to, int delay, int distance);
    bool loadFromCSV(const string& filename);
    const vector<Edge>& getNeighbors(const string& airport) const;
    const unordered_map<string, vector<Edge>>& getAllNodes() const;
};

#endif
