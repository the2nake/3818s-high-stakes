#pragma once

class ChassisController {
public:
  virtual void move(double x, double y, double r) = 0;

protected:
  ChassisController() {}
};