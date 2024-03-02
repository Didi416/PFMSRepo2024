#ifndef CIRCLE_H
#define CIRCLE_H

class Circle
{
  double _radius; //radius
public:
  Circle(int r); //Constructor
  void setRadius(int r); //A Method that sets the radius
  double returnArea(void); //A Method that returns the area
  double returnPerimeter(); //A Method that returns the perimeter
  void setArea(int r); //A Method that sets the area
  void setPerimeter(int r); //A Method that sets the perimeter
};

#endif // CIRCLE_H
