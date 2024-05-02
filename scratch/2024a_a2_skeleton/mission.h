#ifndef MISSION_H
#define MISSION_H

#include <vector>
#include "missioninterface.h"
#include "pfmsconnector.h"

struct GoalsInfo{
  pfms::geometry_msgs::Point point;
  double distanceToGoal;
  double timeToGoal;

};

typedef std::vector<std::vector<std::pair<int,double>> > AdjacencyList;

class Mission: public MissionInterface
{
public:
    /**
    @brief The Mission default constructor, takes in a container of controllers to use for access to Controller class functions
    @sa ControllerInterface and @sa MissionInterface for more information
    */
  Mission(std::vector<ControllerInterface*> controllers);
  ~Mission();
  /**
     * @brief Accepts the container of goals for a specific platform
     *
     * @param goals Vector of goal points that the platform needs to check if it can reach
     */
    void setGoals(std::vector<pfms::geometry_msgs::Point> goals, pfms::PlatformType platform);

    /**
     * @brief Runs the mission
      * @return bool indicating mission complete (false if mission not possible OR aborted because it
      * can not be completed )
     */
    bool run();

    /**
    Returns mission completion status (indicating percentage of completion of task) by each platform @sa setGoals
    @return vector with each element of vector corresponding to a platform. The value is percent of completed distance of entire mission for the corresponding platform value between 0-100.
    */
    std::vector<unsigned int> status(void);

    /**
     * @brief Set mission objective, either BASIC, ADVANCED or SUPER
     */
    void setMissionObjective(mission::Objective objective);

    /**
     * @brief Returns a vector of same size as number of controllers (platforms).
     * The values in the vector correspond to the total distance travelled by the corresponding platform
     * from the time of starting the program.
     *
     * @return std::vector<double> - each element is the distance travelled for each vehicle [m]
     *
     */
    std::vector<double> getDistanceTravelled();

    /**
     * @brief Returns a vector of same size as number of controllers (platforms).
     * The values in the vector correspond to the time the corresponding platfore has been moving
     * from the time the program started. Moving means the platform was not stationary.
     *
     * @return std::vector<double> - each element is the time in motion for each vehicle [m]
     *
     */
    std::vector<double> getTimeMoving();

    /**
     * @brief Returns a vector of same size as number of goals. The values in the vector
     * correspond to the platform number that is completing the goal
     *
     * @return vector of pairs of unsigned int's corresponds to platform number completing the goal (first int is the controller, second is goal number)
     *
     */
    std::vector<std::pair<int, int>> getPlatformGoalAssociation();

    /**
     * @brief Used to generate a graph representing which other goals can be accessed by each goal to be searched to identify the optimal (shortest) path
     * @param controller int corresponding to the position of the current controller
     * @return Adjacency List (graph) representing nodes (goals) and edges (connections/which goals can be reached from other goals)
     *
     */
    AdjacencyList generateGraph(int controller);
    // /**
    //  * @brief Returns a vector of int that is the order determined to be the best (shortest path through all goals)
    //  * @param graph AdjacencyList which is a generated graph showing connecting goals
    //  * @return vector of ints of size corresponding to number of goals, with the order of the shortest path
    //  *
    //  */
    // std::vector<int> bestPathSearch(AdjacencyList graph);

private:
  std::vector<ControllerInterface*> controllers_; //!< A private copy of ControllerInterfaces @sa ControllerInterface
  std::vector<pfms::geometry_msgs::Point> missionGoals_; //!< A private copy of goals
  std::vector<std::pair<int, int>> platGoalAssoc_; //vector of which goals are assigned to a platform and order of goals to be visited
  mission::Objective objective_; //mission objective (BASIC, ADVANCED, SUPER)
  std::vector<unsigned int> status_; //vector for storing percentage of mission completed
  std::vector<double> totalMissionDistance_; //total mission distance between all goals given to platform
  std::vector<double> totalMissionTime_; //total estimated time for mission to complete
  std::vector<double> distancesFromOrigin_; //vector to store distances from the platform origin to all different goals, used to add initial travel distance to first goal for tsp
};

#endif // RANGERFUSION_H
