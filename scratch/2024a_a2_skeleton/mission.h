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

class Mission: public MissionInterface
{
public:
    /**
    The Default constructor
    @sa ControllerInterface and @sa MissionInterface for more information
    */
  Mission(std::vector<ControllerInterface*> controllers);
  /**
     * @brief Accepts the container of goals.
     *
     * @param goals
     */
    void setGoals(std::vector<pfms::geometry_msgs::Point> goals, pfms::PlatformType platform);

    /**
     * @brief Runs the mission
      * @return bool indicating mission complete (false if mission not possible OR aborted because it
      * can not be completed )
     */
    bool run();

    /**
    Retrurns mission completion status (indicating percentage of completion of task) by each platform @sa setGoals
    @return vector with each element of vector corresponding to a platform. The value is percent of completed distance of entire mission for the corresponding platform value between 0-100.
    */
    std::vector<unsigned int> status(void);

    /**
     * @brief Set mission objective
     */
    void setMissionObjective(mission::Objective objective);

    /**
     * @brief Returns a vector of same size as number of controllers (platforms).
     * The values in the vector correspond to the total distance travelled by the corresponding platform
     * from the time of starting the program.
     *
     * @return std::vector<double> - each elemtn distance travelled for each vehicle [m]
     *
     */
    std::vector<double> getDistanceTravelled();

    /**
     * @brief Returns a vector of same size as number of controllers (platforms).
     * The values in the vector correspond to the time the corresponding platfore has been moving
     * from the time the program started. Moving means the platform was not stationary.
     *
     * @return std::vector<double> - each elemtn distance travelled for each vehicle [m]
     *
     */
    std::vector<double> getTimeMoving();

    /**
     * @brief Returns a vector of same size as number of goals. The values in the vector
     * correspond to the platform number that is completing the goal
     *
     * @return vector of unsigned int's corresponds to platform number completing the goal
     *
     * @sa grabAndFuseData
     */
    std::vector<std::pair<int, int>> getPlatformGoalAssociation();

private:
  std::vector<ControllerInterface*> controllers_; //!< A private copy of ControllerInterfaces @sa ControllerInterface
  std::vector<pfms::geometry_msgs::Point> missionGoals_; //!< A private copy of goals
  std::vector<std::pair<int, int>> platGoalAssoc_;
  mission::Objective objective_;
  std::vector<unsigned int> status_;
  std::vector<std::thread> threads_;
  std::vector<double> totalMissionDistance_;
  std::vector<double> totalMissionTime_;
};

#endif // RANGERFUSION_H
