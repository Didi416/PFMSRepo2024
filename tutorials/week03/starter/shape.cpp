#include "shape.h"

Shape::Shape() :
    description_("unkown"), centreX_(0),centreY_(0)
{

}

std::string Shape::getDescription(){
   return description_;
}
