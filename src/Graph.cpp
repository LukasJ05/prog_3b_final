#include "Graph.h"
#include <fstream>
#include <sstream>

void Graph::addEdge(const string& from, const string& to, int delay, int distance)
{
    adj[from].emplace_back(to, delay, distance);
}

bool Graph::loadFromCSV(const string& filename)
{
    ifstream file(filename);
    if (!file.is_open())
    {
        return false;
    }

    string line;
    getline(file, line);
    while (getline(file, line))
    {
        stringstream ss(line);
        string from, to, delayStr, distStr;
        getline(ss, from, ',');
        getline(ss, to, ',');
        getline(ss, delayStr, ',');
        getline(ss, distStr);
        if (from.empty() || to.empty() || delayStr.empty() || distStr.empty())
        {
            continue;
        }

        int delay = stoi(delayStr);
        int distance = stoi(distStr);
        addEdge(from, to, delay, distance);
    }

    return true;
}

const vector<Edge>& Graph::getNeighbors(const string& airport) const
{
    static const vector<Edge> empty;
    auto it = adj.find(airport);
    return it != adj.end() ? it->second : empty;
}

const unordered_map<string, vector<Edge>>& Graph::getAllNodes() const
{
    return adj;
}