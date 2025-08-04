#include "Graph.h"
#include "Pathfinder.h"
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <fstream>
#include <sstream>
using namespace std;

//airport codes to US states
unordered_map<string, string> airportToState =
{
    {"ATL", "GA"}, {"LAX", "CA"}, {"ORD", "IL"}, {"DFW", "TX"}, {"DEN", "CO"},
    {"JFK", "NY"}, {"SFO", "CA"}, {"SEA", "WA"}, {"LAS", "NV"}, {"MCO", "FL"},
    {"CLT", "NC"}, {"PHX", "AZ"}, {"MIA", "FL"}, {"IAH", "TX"}, {"BOS", "MA"},
    {"MSP", "MN"}, {"FLL", "FL"}, {"DTW", "MI"}, {"PHL", "PA"}, {"BWI", "MD"},
    {"SLC", "UT"}, {"SAN", "CA"}, {"DCA", "DC"}, {"TPA", "FL"}, {"PDX", "OR"},
    {"HNL", "HI"}, {"STL", "MO"}, {"MDW", "IL"}, {"AUS", "TX"}, {"RDU", "NC"},
    {"OAK", "CA"}, {"MSY", "LA"}, {"SMF", "CA"}, {"SJC", "CA"}, {"CLE", "OH"},
    {"PIT", "PA"}, {"MKE", "WI"}, {"IND", "IN"}, {"CMH", "OH"}, {"CVG", "KY"}
};

class App
{
private:
    Graph graph;
    Pathfinder* pathfinder;
    bool dataLoaded = false;

public:
    App()
    {
        dataLoaded = graph.loadFromCSV("data/flights_real_usa.csv");
        if (!dataLoaded)
        {
            cerr << "Error: Could not load flight data. Exiting.\n";
            exit(1);
        }
        pathfinder = new Pathfinder(graph);
    }

    ~App()
    {
        delete pathfinder;
    }

    void listAirportsByState(const string& state)
    {
        unordered_set<string> airports;
        for (const auto& [airport, _] : graph.getAllNodes())
        {
            if (airportToState.count(airport) && airportToState[airport] == state)
            {
                airports.insert(airport);
            }
        }

        if (airports.empty())
        {
            cout << "No airports found for state: " << state << "\n";
        } else
        {
            cout << "Airports in " << state << ": ";
            for (const string& a : airports) cout << a << " ";
            cout << "\n";
        }
    }

    void run()
    {
        while (true)
        {
            int choice;
            cout << "\n==== Airport Delay Path Finder ====" << endl;
            cout << "1. Find shortest path\n2. List airports by state\n0. Exit\nChoice: ";
            cin >> choice;
            if (choice == 0) break;
            else if (choice == 1)
            {
                string start, end;
                int algo;
                cout << "Enter starting airport code: "; cin >> start;
                cout << "Enter destination airport code: "; cin >> end;
                cout << "Choose algorithm:\n1. Dijkstra\n2. A*\nEnter choice: ";
                cin >> algo;
                if (algo == 1) pathfinder->dijkstra(start, end);
                else if (algo == 2) pathfinder->astar(start, end);
                else cout << "Invalid algorithm choice.\n";
            }
            else if (choice == 2)
            {
                string state;
                cout << "Enter 2-letter state code (e.g., CA): ";
                cin >> state;
                listAirportsByState(state);
            } else
            {
                cout << "Invalid menu choice.\n";
            }
        }
    }
};

int main()
{
    App app;
    app.run();
    return 0;
}
