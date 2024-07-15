#include "subzerolib/api/filter/kalman-filter.hpp"
#include <memory>

void KalmanFilter::predict(int delta_ms, Eigen::VectorXd control_input) {
  if (control_input.rows() != nu) {
    return;
  }
  next_state = state_transition_matrix * state + control_matrix * control_input;
  next_covariance = state_transition_matrix * covariance *
                        state_transition_matrix.transpose() +
                    process_noise_covariance;
}

void KalmanFilter::update(int delta_ms, Eigen::VectorXd measurement) {
  if (measurement.rows() != nz) {
    return;
  }
  Eigen::MatrixXd kalman_gain =
      (next_covariance * observation_matrix.transpose()) *
      (observation_matrix * next_covariance * observation_matrix.transpose() +
       measurement_covariance)
          .inverse();
  state = next_state +
          kalman_gain * (measurement - observation_matrix * next_state);
  auto identity = Eigen::MatrixXd::Identity(nx, nx);
  auto ikh = identity - kalman_gain * observation_matrix;
  covariance = (ikh * next_covariance * ikh.transpose()) +
               kalman_gain * measurement_covariance * kalman_gain.transpose();
}

using Builder = KalmanFilter::Builder;
using Matrix = Eigen::MatrixXd;
using Vector = Eigen::VectorXd;

Builder &Builder::with_state_transition_matrix(Matrix f) {
  if (f.rows() == nx && f.cols() == 1) {
    b_f = f;
  }

  return *this;
}

Builder &Builder::with_control_matrix(Matrix g) {
  if (g.rows() == nx && g.cols() == nu) {
    b_g = g;
  }

  return *this;
}

Builder &Builder::with_observation_matrix(Matrix h) {
  if (h.rows() == nz && h.cols() == nx) {
    b_h = h;
  }

  return *this;
}

Builder &Builder::with_process_noise_covariance(Matrix q) {
  if (q.rows() == q.cols() && q.rows() == nx) {
    b_q = q;
  }

  return *this;
}

Builder &Builder::with_measurement_covariance(Matrix r) {
  if (r.rows() == r.cols() && r.rows() == nz) {
    b_r = r;
  }

  return *this;
}

Builder &Builder::with_initial_state(Vector x) {
  if (x.rows() == nx && x.cols() == 1) {
    b_x = x;
  }

  return *this;
}

Builder &Builder::with_initial_covariance(Matrix p) {
  if (p.rows() == p.cols() && p.rows() == nx) {
    b_p = p;
  }

  return *this;
}

std::shared_ptr<KalmanFilter> Builder::build() {
  if (!(initialised(b_f) && (nu == 0 || initialised(b_g)) && initialised(b_h) &&
        initialised(b_q) && initialised(b_r) && initialised(b_x) &&
        initialised(b_p))) {
    return nullptr;
  }

  std::shared_ptr<KalmanFilter> filter{
      new KalmanFilter(nx, nu, nz, b_f, b_g, b_h, b_q, b_r)};
  filter->state = b_x;
  filter->covariance = b_p;

  return filter;
}