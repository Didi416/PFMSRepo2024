#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "controllerinterface.h"
#include "pfmsconnector.h"

class Controller: public ControllerInterface
{
public:
  //Default constructors should set all attributes to a default value
  Controller();
  ~Controller();

  //See controllerinterface.h for more information
};

#endif // CONTROLLER_H
