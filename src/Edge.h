

#ifndef EDGE_H
#define EDGE_H

#include <string>
using namespace std;

class Edge {
public:
    string to;
    int avgDelay;
    int distance;
    Edge(string to, int avgDelay, int distance);
};

#endif //EDGE_H
