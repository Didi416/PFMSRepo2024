#include <iostream>
#include <vector>
#include <stack>
#include <set>
#include <queue>
#include <map>
#include <algorithm>

using namespace std;

//! A container for the adjacency list
typedef vector<vector<int> > AdjacencyList;

//! Students should attempt to create a function that prints the adjacency list
void printGraph(AdjacencyList graph){
  int node_num=0;
  for (auto node : graph){
    std::cout << node_num++ << " : ";
    for (auto edge : node){
      std::cout << edge << " " ;
    }
      std::cout << std::endl;
  }

}

AdjacencyList floorPlan(){
  AdjacencyList floorPlanGraph = {
    {2, 4}, //Room 1 connects to 2 and 4
    {1, 7}, //Room 2 connects to 1 and 7
    {4}, //Room 3 connects to 4
    {1, 3, 5, 6}, //Room 4 connects to 1, 3, 5 and 6
    {4}, //Room 5 connects to 4
    {4, 7}, //Room 6 connects to 4 and 7
    {2, 6}, //Room 7 connects to 2 and 6
  };
  return floorPlanGraph;
}

int main () {
    // Build the graph
    AdjacencyList example_graph = {
        {1, 2},     // Node 0 is connected to nodes 1 and 2 (via edges)
        {0, 4},     // Node 1 is connected to nodes 0 and 4 (via edges)
        {0, 3, 4},  // Node 2 is connected to nodes 0, 3 and 4 (via edges)

        //! Complete for remaining nodes 3,4,5 and 6
        {2, 4, 5},  // Node 3 is connected to nodes 2, 4 and 5 (via edges)
        {1, 2, 3},  // Node 4 connected to nodes 1, 2, 3
        {3, 6},     // Node 5 connected to nodes 3 and 6
        {5}         // Node 6 connected to 5 only
    };

    std::cout<<"Example Graph"<<std::endl;
    printGraph(example_graph);
    std::cout<<"Floor Plan"<<std::endl;
    printGraph(floorPlan());


    return 0;
}
