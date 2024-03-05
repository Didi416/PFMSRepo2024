#ifndef SHAPE_H
#define SHAPE_H

#include <string>
#include <vector>

/*!
 *  \ingroup   ac_shapre Shape
 *  \brief     Shape base class
 *  \details
 *  This class is the base class for all shapes.\n
 */
class Shape
{
public:
    Shape();
    void setCentre(double x, double y);
    void offsetCentre(double x, double y);
    std::string getDescription();
    double getArea();

    double getcentreX();
    double getcentreY();
    double getCentreDist();

    virtual bool intersectionCheck(double pointX, double pointY) = 0;
    std::vector<Shape *> intersectingShapes(std::vector<Shape *> shapeVec, double pointX, double pointY);

protected:
    std::string description_;//!< description of shape
private:
    double centreX_;//!< X coordinate of centre of shape
    double centreY_;//!< Y coordinate of centre of shape
};

#endif // SHAPE_H
