#include "subzerolib/api/filter/kalman-filter.hpp"
#include <iostream>

int main() {
  Eigen::Vector<double, 6> initial_state{
      {0, 0, 0, 0, 0, 0}
  };
  Eigen::Matrix<double, 6, 6> initial_covariance;
  initial_covariance.setZero();
  initial_covariance.diagonal() = Eigen::Vector<double, 6>{
      {500, 500, 500, 500, 500, 500}
  };

  const double dt = 1;
  Eigen::Matrix<double, 6, 6> state_transition_matrix{
      {1, dt, .5 * dt * dt, 0,  0,            0},
      {0,  1,           dt, 0,  0,            0},
      {0,  0,            1, 0,  0,            0},
      {0,  0,            0, 1, dt, .5 * dt * dt},
      {0,  0,            0, 0,  1,           dt},
      {0,  0,            0, 0,  0,            1},
  };
  Eigen::Matrix<double, 2, 6> observation_matrix{
      {1, 0, 0, 0, 0, 0},
      {0, 0, 0, 1, 0, 0},
  };

  const double s_l = 0.2; // linear acceleration stdev
  const double v_l = s_l * s_l;

  const double c4 = 0.25 * dt * dt * dt * dt;
  const double c3 = 0.5 * dt * dt * dt;
  const double c2 = dt * dt;

  Eigen::Matrix<double, 6, 6> process_noise_covariance{
      { c4,  c3,  c2, 0.0, 0.0, 0.0},
      { c3,  c2,  dt, 0.0, 0.0, 0.0},
      { c3,  dt,   1, 0.0, 0.0, 0.0},
      {0.0, 0.0, 0.0,  c4,  c3,  c2},
      {0.0, 0.0, 0.0,  c3,  c2,  dt},
      {0.0, 0.0, 0.0,  c2,  dt,   1}
  };
  process_noise_covariance = process_noise_covariance * v_l;

  Eigen::Matrix2d measurement_covariance{
      {  9, 0.0},
      {0.0,   9}
  };

  auto filter = KalmanFilter::Builder(6, 0, 2)
                    .with_initial_state(initial_state)
                    .with_initial_covariance(initial_covariance)
                    .with_measurement_covariance(measurement_covariance)
                    .with_state_transition_matrix(state_transition_matrix)
                    //.with_control_matrix()
                    .with_observation_matrix(observation_matrix)
                    .with_process_noise_covariance(process_noise_covariance)
                    .build();
  Eigen::Matrix<double, 2, 1> meas{{301.5}, {-401.46}};
  if (filter == nullptr) {
    std::cout << "filter init failed" << std::endl;
    return 0;
  }

  filter->predict(1000);
  filter->update(1000, meas);

  std::cout << "state" << std::endl;
  std::cout << filter->get_state() << std::endl;
  std::cout << "covariance" << std::endl;
  std::cout << filter->get_covariance() << std::endl;

  /*
  expected:
   299.107
   199.408
    66.473
  -398.274
  -265.521
   -88.5187

  8.92857 5.95249 1.98443       0       0       0
  5.95249 503.986 334.667       0       0       0
  1.98427  334.68 444.913       0       0       0
        0       0       0 8.92857 5.95249 1.98443
        0       0       0 5.95249 503.986 334.667
        0       0       0 1.98443 334.667 444.908
  */

  return 0;
}