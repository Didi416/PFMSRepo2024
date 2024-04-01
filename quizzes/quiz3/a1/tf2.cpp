#include "tf2.h"
#include "tf.h"
#include <cmath> // for trig operations

namespace tf2 {
    Point rotateFrame(Point point, double theta){
      Point rotatedPoint;
      rotatedPoint.x = point.x * cos(theta) - point.y * sin(theta);
      rotatedPoint.y = point.x * sin(theta) + point.y * cos(theta);
      rotatedPoint.z = point.z;
      return rotatedPoint;
    }

    Point transformLocalToGlobal(Point localPoint, Point localOrigin, double rotation){
      Point rotatedPoint = rotateFrame(localPoint, rotation);
      Point globalPoint;
      globalPoint.x = rotatedPoint.x + localOrigin.x;
      globalPoint.y = rotatedPoint.y + localOrigin.y;
      globalPoint.z = rotatedPoint.z + localOrigin.z;
      return globalPoint;
    }

    Point transformGlobalToLocal(Point globalPoint, Point localOrigin, double rotation){
      Point localPoint;
      localPoint.x = globalPoint.x - localOrigin.x;
      localPoint.y = globalPoint.y - localOrigin.y;
      localPoint.z = globalPoint.z - localOrigin.z;
      Point rotatedPoint = rotateFrame(localPoint, rotation);
      return rotatedPoint;
    }

    //! @todo
    //! TASK 1 - Refer to README.md and the Header file for full description
    Point local2Global(RangeBearingStamped rangeBearing, Pose aircraft)
    {
        Point p;
        double x = rangeBearing.range*cos(rangeBearing.bearing);
        double y = rangeBearing.range*sin(rangeBearing.bearing);
        Point localPoint = {x,y,0};
        Point localOrigin = aircraft.position;
        double rotation = tf::quaternionToYaw(aircraft.orientation);

        double normRotation = normaliseAngle(rotation);
        p = transformLocalToGlobal(localPoint,localOrigin,normRotation);
        return p;
    }

    //! @todo
    //! TASK 2 - Refer to README.md and the Header file for full description
    RangeBearingStamped global2local(Point globalEnemy, Pose aircraft)
    {
        RangeBearingStamped rbstamped = {0, 0, 0};
        Point localOrigin = aircraft.position;
        double rotation = tf::quaternionToYaw(aircraft.orientation);
        double normRotation = normaliseAngle(rotation);
        
        Point localEnemy = transformGlobalToLocal(globalEnemy,localOrigin,-normRotation);
        rbstamped.range = sqrt(pow(localEnemy.x, 2) + pow(localEnemy.y, 2));
        rbstamped.bearing = atan2(localEnemy.y, localEnemy.x);
        return rbstamped;
    }

    double normaliseAngle(double theta) {
      if (theta > (2 * M_PI))
        theta = theta - (2 * M_PI);
      else if (theta < 0)
        theta = theta + (2 * M_PI);

      if (theta > M_PI){
          theta = -( (2* M_PI) - theta);
      }

      return theta;
    }

}
