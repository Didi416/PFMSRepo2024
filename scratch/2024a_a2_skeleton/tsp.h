#ifndef TSP_H
#define TSP_H


#include <cmath>
#include <thread>
#include <vector>

typedef std::vector<std::vector<std::pair<int,double>> > AdjacencyList;

class TSP
{
    TSP();
    ~TSP();

    AdjacencyList generateGraph(int controller);
    std::vector<int> bestPathSearch(AdjacencyList graph);

    public:

    private:
};
#endif //TSP_H