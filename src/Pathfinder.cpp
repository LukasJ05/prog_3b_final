#include "Pathfinder.h"
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <limits>
#include <iostream>
#include <chrono>
#include <cmath>
using namespace std;
using namespace std::chrono;

constexpr double EARTH_RADIUS_MILES = 3958.8;

static double deg2rad(double deg)
{
    return deg * M_PI / 180.0;
}

static double haversine(double lat1, double lon1, double lat2, double lon2)
{
    double dLat = deg2rad(lat2 - lat1);
    double dLon = deg2rad(lon2 - lon1);
    lat1 = deg2rad(lat1);
    lat2 = deg2rad(lat2);

    double a = sin(dLat / 2) * sin(dLat / 2) +
               sin(dLon / 2) * sin(dLon / 2) * cos(lat1) * cos(lat2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return EARTH_RADIUS_MILES * c;
}

// airport latitude, longitude
static const unordered_map<string, pair<double, double>> airportCoordinates =
{
    {"ATL", {33.6407, -84.4277}},
    {"LAX", {33.9416, -118.4085}},
    {"ORD", {41.9742, -87.9073}},
    {"DFW", {32.8998, -97.0403}},
    {"DEN", {39.8561, -104.6737}},
    {"JFK", {40.6413, -73.7781}},
    {"SFO", {37.6213, -122.3790}},
    {"SEA", {47.4502, -122.3088}},
    {"LAS", {36.0840, -115.1537}},
    {"MCO", {28.4312, -81.3081}},
    {"CLT", {35.2144, -80.9473}},
    {"PHX", {33.4342, -112.0116}},
    {"MIA", {25.7959, -80.2871}},
    {"IAH", {29.9902, -95.3368}},
    {"BOS", {42.3656, -71.0096}},
    {"MSP", {44.8848, -93.2223}},
    {"FLL", {26.0726, -80.1527}},
    {"DTW", {42.2162, -83.3554}},
    {"PHL", {39.8729, -75.2437}},
    {"BWI", {39.1754, -76.6684}},
    {"SLC", {40.7884, -111.9778}},
    {"SAN", {32.7338, -117.1933}},
    {"DCA", {38.8521, -77.0377}},
    {"TPA", {27.9755, -82.5332}},
    {"PDX", {45.5887, -122.5975}},
    {"HNL", {21.3187, -157.9225}},
    {"STL", {38.7487, -90.3700}},
    {"MDW", {41.7868, -87.7522}},
    {"AUS", {30.1945, -97.6699}},
    {"RDU", {35.8776, -78.7875}},
    {"OAK", {37.7213, -122.2216}},
    {"MSY", {29.9939, -90.2580}},
    {"SMF", {38.6951, -121.5908}},
    {"SJC", {37.3639, -121.9289}},
    {"CLE", {41.4128, -81.8498}},
    {"PIT", {40.4915, -80.2329}},
    {"MKE", {42.9472, -87.8966}},
    {"IND", {39.7173, -86.2944}},
    {"CMH", {39.9980, -82.8916}},
    {"CVG", {39.0489, -84.6677}}
};

Pathfinder::Pathfinder(const Graph& g) : graph(g) {}

double Pathfinder::heuristic(const string& from, const string& to) const
{
    auto itFrom = airportCoordinates.find(from);
    auto itTo = airportCoordinates.find(to);

    if (itFrom == airportCoordinates.end() || itTo == airportCoordinates.end())
    {
        return 0.0;
    }

    double lat1 = itFrom->second.first;
    double lon1 = itFrom->second.second;
    double lat2 = itTo->second.first;
    double lon2 = itTo->second.second;

    return haversine(lat1, lon1, lat2, lon2);
}

vector<Edge> Pathfinder::dijkstra(const string& start, const string& end)
{
    auto startTime = high_resolution_clock::now();

    unordered_map<string, double> distances;
    unordered_map<string, string> previous;
    unordered_set<string> visited;
    priority_queue<pair<double, string>, vector<pair<double, string>>, greater<>> pq;

    for (const auto& pair : graph.getAllNodes())
        distances[pair.first] = numeric_limits<double>::infinity();
    distances[start] = 0;

    pq.emplace(0, start);

    while (!pq.empty()) {
        auto [curDelay, airport] = pq.top(); pq.pop();
        if (visited.count(airport)) continue;
        visited.insert(airport);

        if (airport == end) break;

        for (const Edge& edge : graph.getNeighbors(airport))
        {
            double newDelay = curDelay + edge.avgDelay;
            if (newDelay < distances[edge.to])
            {
                distances[edge.to] = newDelay;
                previous[edge.to] = airport;
                pq.emplace(newDelay, edge.to);
            }
        }
    }

    auto endTime = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(endTime - startTime).count();

    vector<Edge> path;
    string current = end;
    while (previous.find(current) != previous.end())
    {
        string prev = previous[current];
        for (const Edge& edge : graph.getNeighbors(prev))
        {
            if (edge.to == current)
            {
                path.insert(path.begin(), edge);
                break;
            }
        }
        current = prev;
    }

    cout << "\nDijkstra's Path:\n";
    double totalDelay = 0, totalDist = 0;
    for (const Edge& edge : path)
    {
        cout << edge.to << " <- ";
        totalDelay += edge.avgDelay;
        totalDist += edge.distance;
    }
    cout << start << "\n";
    cout << "Total Delay: " << totalDelay << " minutes\n";
    cout << "Total Distance: " << totalDist << " miles\n";
    cout << "Number of Hops: " << path.size() << "\n";
    cout << "Nodes Explored (Dijkstra): " << visited.size() << "\n";
    cout << "Execution Time: " << duration << " ms\n";

    return path;
}

vector<Edge> Pathfinder::astar(const string& start, const string& end)
{
    auto startTime = high_resolution_clock::now();

    unordered_map<string, double> gScore, fScore;
    unordered_map<string, string> previous;
    unordered_set<string> visited;
    priority_queue<pair<double, string>, vector<pair<double, string>>, greater<>> pq;

    for (const auto& pair : graph.getAllNodes())
    {
        gScore[pair.first] = numeric_limits<double>::infinity();
        fScore[pair.first] = numeric_limits<double>::infinity();
    }
    gScore[start] = 0;
    fScore[start] = heuristic(start, end);

    pq.emplace(fScore[start], start);

    while (!pq.empty())
    {
        auto [_, airport] = pq.top(); pq.pop();
        if (visited.count(airport)) continue;
        visited.insert(airport);

        if (airport == end) break;

        for (const Edge& edge : graph.getNeighbors(airport))
        {
            double tentative_gScore = gScore[airport] + edge.avgDelay;
            if (tentative_gScore < gScore[edge.to])
            {
                gScore[edge.to] = tentative_gScore;
                fScore[edge.to] = tentative_gScore + heuristic(edge.to, end);
                previous[edge.to] = airport;
                pq.emplace(fScore[edge.to], edge.to);
            }
        }
    }

    auto endTime = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(endTime - startTime).count();

    vector<Edge> path;
    string current = end;
    while (previous.find(current) != previous.end())
    {
        string prev = previous[current];
        for (const Edge& edge : graph.getNeighbors(prev))
        {
            if (edge.to == current)
            {
                path.insert(path.begin(), edge);
                break;
            }
        }
        current = prev;
    }

    cout << "\nA* Path:\n";
    double totalDelay = 0, totalDist = 0;
    for (const Edge& edge : path)
    {
        cout << edge.to << " <- ";
        totalDelay += edge.avgDelay;
        totalDist += edge.distance;
    }
    cout << start << "\n";
    cout << "Total Delay: " << totalDelay << " minutes\n";
    cout << "Total Distance: " << totalDist << " miles\n";
    cout << "Number of Hops: " << path.size() << "\n";
    cout << "Nodes Explored (A*): " << visited.size() << "\n";
    cout << "Execution Time: " << duration << " ms\n";

    return path;
}