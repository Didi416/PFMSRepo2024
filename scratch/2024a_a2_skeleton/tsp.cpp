#include "tsp.h"
#include <vector>
#include <algorithm>


TSP::TSP(){}

TSP::~TSP(){}

std::vector<int> TSP::bestPathSearch(AdjacencyList graph, std::vector<double> distances){ // Search the goals graph to find shortest path
    //start tsp search, focusing on moving to the goal with the next shortest distance from current goal
    std::vector<int> order;
    //List all goals in a vector, to do permutations
    std::vector<int> goals;
    for (unsigned int i=0; i< graph.size(); i++){
        goals.push_back(i);
    }
    double minDistance = 1e6; //Set initial minimum distance to a large value, so first permutation is definitely less than

    //Go through all permutations of the order of goals, comparing total distances to 
    //determine the minimum distance possible and asociated permuation:
    do {
        bool OK = true; //Use this to abort search if distanec is already above minimum distance
        unsigned int i = 1;//Start from index 1 to end 
        double dist = 0; // Current distance travelled through goals
        while((i<goals.size()) && (OK)){
            // Checks distance between two goals in the order dictated by the current permutation
            unsigned int goal1 = goals.at(i-1);
            unsigned int goal2 = goals.at(i);
            //We find in the adjacency list (graph), the goal1 connection to goal2 and access the second element which is the distance between goal1 and goal2
            dist += graph.at(goal1).at(goal2).second;
            //We abort current search if dist is already over the min distance
            if(dist > minDistance){
                OK = false;
            }
            i++; // increment to next goal in permutation
        }
        dist += distances.at(goals.at(0));
        if(dist<minDistance){
            minDistance=dist; // Save current dist as new minimum distance
            order.clear(); // Clear the current order of goals
            order=goals; // Save the order of goals as current shortest path
        }
    } 
    while (std::next_permutation(goals.begin(), goals.end())); //Go to next permutation of goals order

    return order;
}

AdjacencyList TSP::generateGraph(std::vector<tsp::Point> goals){ //Generate graph for searching from mission goals
    AdjacencyList graph (goals.size()); // Create graph of size of mission goals (number of vectors in the vector = missionGoals.size())
    double dist; // initialise for calculating distance
    tsp::Point origin; //stores origin or starting position

    for (int i=0; i<goals.size(); i++){ //iterate through goals
        // store current goal point as the current origin to calculate the distances from the current goal to all other goals
        origin.x = goals.at(i).x;
        origin.y = goals.at(i).y;
        origin.z = goals.at(i).z;
        for (int j=0; j<goals.size(); j++){ //iterate through goals
            dist = sqrt(pow(origin.x - goals.at(j).x, 2) + pow(origin.y - goals.at(j).y, 2)); //calculates straight line distance to goals, as Audi library doesn't always calculate the distance correctly
             graph.at(i).push_back(std::make_pair(j,dist)); //add an entry into the graph, to signify that the goal 'j' can be reached from goal 'i' in a distance calculated by 'dist'
        }
    }
    return graph;
}