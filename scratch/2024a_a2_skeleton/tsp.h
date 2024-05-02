#ifndef TSP_H
#define TSP_H


#include <cmath>
#include <thread>
#include <vector>

typedef std::vector<std::vector<std::pair<int,double>> > AdjacencyList;

class TSP
{
    public:
        TSP();
        ~TSP();

        // AdjacencyList generateGraph(int controller);
        std::vector<int> bestPathSearch(AdjacencyList graph, std::vector<double> distances);

    private:

};
#endif //TSP_H