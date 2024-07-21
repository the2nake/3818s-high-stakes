#pragma once

#include <eigen/Dense>

class Filter {
public:
  virtual void predict(int delta_ms = 0) = 0;
  virtual void predict(Eigen::VectorXd control_input, int delta_ms = 0) = 0;
  virtual void update(Eigen::VectorXd measurement, int delta_ms = 0) = 0;

  virtual Eigen::VectorXd get_state() = 0;
  virtual Eigen::MatrixXd get_covariance() = 0;

  virtual void initialise(Eigen::VectorXd state,
                          Eigen::MatrixXd covariance) = 0;

protected:
  Filter() {}
};