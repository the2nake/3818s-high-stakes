#pragma once

class ExitCondition {
public:
  virtual void append(ExitCondition &condition) = 0;
  virtual bool is_met() = 0;

protected:
  ExitCondition() {}
};