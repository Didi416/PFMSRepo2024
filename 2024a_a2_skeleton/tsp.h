#ifndef TSP_H
#define TSP_H


#include <cmath>
#include <thread>
#include <vector>


namespace tsp{ 
    /**
     * @brief Struct to contain x, y and z components of goal points, so tsp library can access independantly of other classes/libraries
    */
    struct Point{
        double x;/*!< position x [m] */
        double y;/*!< position y [m] */
        double z;/*!< position z[m] */
    };
}
    
typedef std::vector<std::vector<std::pair<int,double>> > AdjacencyList;

/*!
 *  \brief     TSP Class/Library
 *  \details
 *  Used to generate a graph connecting goal points and then search for the shortest path through. 
 *  \author    Dyandra Prins
 *  \date      2024-02-05
 */

class TSP
{
    public:
        /**
         * @brief Default constructor should set all attributes to a default value,
         * as well as assigning values to private/protected variables from the class header file.
         */
        TSP();
        ~TSP();

        /**
         * @brief Used to generate a graph representing which other goals can be accessed by each goal to be searched to identify the optimal (shortest) path
         * @param goals inputs a vector of points (x,y,z) to generate a graph from
         * @return Adjacency List (graph) representing nodes (goals) and edges (connections/which goals can be reached from other goals)
         *
         */
        AdjacencyList generateGraph(std::vector<tsp::Point> goals);

        /**
         * @brief Returns a vector of int that is the order determined to be the best (shortest path through all goals)
         * @param graph AdjacencyList which is a generated graph showing connecting goals
         * @param distances vector of distances from starting point to all goals
         * @return vector of ints of size corresponding to number of goals, with the order of the shortest path
         *
         */
        std::vector<int> bestPathSearch(AdjacencyList graph, std::vector<double> distances);

};
#endif //TSP_H