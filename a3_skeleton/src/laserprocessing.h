#ifndef LASERPROCESSING_H
#define LASERPROCESSING_H

#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/utils.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <math.h>
#include <mutex>
#include <set>

/*!
 *  \brief     Laser Processing Class
 *  \details
 *  Class to process all laser scan data, reads in current scan and performs functions and calculations to determine cone positions and goals
 *  \author    Dyandra Prins
 *  \date      2024-31-05
 */

class LaserProcessing
{
public:
  /*! @brief Constructor that allocates internals
   *
   *  @param[in]    laserScan - laserScan to be processed
   */
  LaserProcessing(sensor_msgs::msg::LaserScan laserScan);

  /*! @brief Count number of readings belonging to objects (not infinity, nan or max range) from the last
  * laser scan provided (either via @sa newScan or @sa LaserProcessing constructor)
  * thread-safe function, internally creates a copy fo laserScan_ 
  *
  * @return the number of laser readings that belong to objects
  */
  unsigned int countObjectReadings();

   /*!
   *  @brief Count number of segments in current laser scan,
   *  @note Segments are formed by high intensity readings.
   * A segment is a sequence (consecutive) high intensity readings that are less than 0.3m
   *
   * @return the number of segments in the current laser scan
   */
  unsigned int countSegments();
  
  /*!
   *  @brief Count number of laser scane readings close to the front of the car, and checks if number is excessively large
   *  @note A large object is determined by extremely large laser scane readings
   *
   * @return the number of segments in the current laser scan
   */
  bool detectLargeObstacle();

  /*! @brief Accepts a new laserScan, threadsafe function
   *  @param[in]    laserScan  - laserScan supplied
   */
  void newScan(sensor_msgs::msg::LaserScan laserScan);

  /*! 
   * @brief Return positions of cone centres
   *
   * The position should be the singular point location of the cone (which can be computed as the average laser readings belonging to the cone/segment)
   * As the cone is circular, the laser will pick up a few readings at the location of the cone.
   * @return vector of points of cone locations (local frame)
   */
  std::vector<geometry_msgs::msg::Point> detectConeCentres();

  /*!
   * @brief Return the locations of the road centre of the current laser scan
   *
   * Detect two cones, that are closest together, and on either side of the road
   * the road centre should be the point in the middle of the two cones
   * @param[in] points - vector of points determined to be cones from detectConeCentres
   * @return vector of points - the locations of the centre of the road
   */
  std::vector<std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>> detectRoad(std::vector<geometry_msgs::msg::Point> points);

private:

  /*! @brief Returns the cartesian position of laer reading at specific index
   * converted from polar coordinats stored in the #laserScan_
   *  @param[in] index - the reading needing conversion
   *  @return position cartesian values
   */
   geometry_msgs::msg::Point polarToCart(unsigned int index);

  /*! 
   * @brief Converts segments (multiple laser readings) to one centre point
   *
   * @param[in] i - accesses ith position of private data member obstacles vector (current segment)
   * @return point - the location of the centre of the segment
   */
  geometry_msgs::msg::Point segmentToPoint(int i);

private:
  sensor_msgs::msg::LaserScan laserScan_;//!< stores current laser scan being processed
  std::mutex mtx; //!< Mutex to protect the laserScan_ from being accessed by multiple threads
  unsigned int objectReadings_; //!< Number of readings belonging to objects
  std::vector<unsigned int> objectReadingIndex_; //!< stores which laser scan readings are closer than max range (not nan or inf)

  std::vector<std::vector<int>> obstacles_; //!< storage of objects detected (by segments)
};

#endif // LASERPROCESSING_H
