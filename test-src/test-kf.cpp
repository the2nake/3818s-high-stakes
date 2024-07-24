#include "subzerolib/api/filter/kalman-filter.hpp"
#include <iostream>
#include <thread>

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
  if (filter == nullptr) {
    std::cout << "filter init failed" << std::endl;
    return 0;
  }

  std::vector<double> meas_x = {
      301.5, 298.23, 297.83, 300.42, 301.94, 299.5, 305.98, 301.25};
  std::vector<double> meas_y = {
      -401.46, -375.44, -346.15, -320.2, -300.08, -274.12, -253.45, -226.4};
  for (int i = 0; i < meas_x.size() && i < meas_y.size(); ++i) {
    filter->predict();
    std::cout << "prediction" << std::endl;
    std::cout << filter->get_pred_state() << std::endl;
    std::cout << filter->get_pred_covariance() << std::endl;
    std::cout << "updated" << std::endl;
    filter->update(Eigen::Vector<double, 2>{meas_x[i], meas_y[i]});
    std::cout << filter->get_state() << std::endl;
    std::cout << filter->get_covariance() << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  }

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
