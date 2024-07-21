#include "subzerolib/api/filter/kalman-filter.hpp"
#ifdef TARGET_V5
#include "subzerolib/api/util/logging.hpp"
#else
#include <iostream>
#endif
#include <memory>

void KalmanFilter::predict(int delta_ms) {
#ifdef TARGET_V5
  if (mutex.take(5)) {
#endif
    next_state = state_transition_matrix * state;
    next_covariance = state_transition_matrix * covariance *
                          state_transition_matrix.transpose() +
                      process_noise_covariance;
#ifdef TARGET_V5
    if (!mutex.give()) {
      subzero::error("[e]: predict(): kf mutex failed to return");
    }
  } else {
    subzero::error("[e]: predict(): kf mutex failed to take");
  }
#endif
}

void KalmanFilter::predict(Eigen::VectorXd control_input, int delta_ms) {
  if (control_input.rows() != nu) {
    return;
  }
#ifdef TARGET_V5
  if (mutex.take(5)) {
#endif
    next_state =
        state_transition_matrix * state + control_matrix * control_input;
    next_covariance = state_transition_matrix * covariance *
                          state_transition_matrix.transpose() +
                      process_noise_covariance;
#ifdef TARGET_V5
    if (!mutex.give()) {
      subzero::error("[e]: predict(): kf mutex failed to return");
    }
  } else {
    subzero::error("[e]: predict(): kf mutex failed to take");
  }
#endif
}

void KalmanFilter::update(Eigen::VectorXd measurement, int delta_ms) {
  if (measurement.rows() != nz) {
    return;
  }

#ifdef TARGET_V5
  if (mutex.take(5)) {
#endif
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
#ifdef TARGET_V5
    if (!mutex.give()) {
      subzero::error("[e]: update(): kf mutex failed to return");
    }
  } else {
    subzero::error("[e]: update(): kf mutex failed to take");
  }
#endif
}

using Builder = KalmanFilter::Builder;
using Matrix = Eigen::MatrixXd;
using Vector = Eigen::VectorXd;

Builder &Builder::with_state_transition_matrix(Matrix f) {
  if (f.rows() == nx && f.cols() == nx) {
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
#ifdef TARGET_V5
    subzero::log("[e]: filter initialisation failed");
    subzero::log("[i]: f_init: %s", initialised(b_f) ? "true" : "false");
    subzero::log("[i]: g_init: %s", initialised(b_g) ? "true" : "false");
    subzero::log("[i]: h_init: %s", initialised(b_h) ? "true" : "false");
    subzero::log("[i]: q_init: %s", initialised(b_q) ? "true" : "false");
    subzero::log("[i]: r_init: %s", initialised(b_r) ? "true" : "false");
    subzero::log("[i]: x_init: %s", initialised(b_x) ? "true" : "false");
    subzero::log("[i]: p_init: %s", initialised(b_p) ? "true" : "false");
#else
    std::cout << "filter init failed" << std::endl;
    std::cout << "f init:" << initialised(b_f) << std::endl;
    std::cout << "g init:" << initialised(b_g) << std::endl;
    std::cout << "h init:" << initialised(b_h) << std::endl;
    std::cout << "q init:" << initialised(b_q) << std::endl;
    std::cout << "r init:" << initialised(b_r) << std::endl;
    std::cout << "x init:" << initialised(b_x) << std::endl;
    std::cout << "p init:" << initialised(b_p) << std::endl;
#endif
    return nullptr;
  }

  std::shared_ptr<KalmanFilter> filter{
      new KalmanFilter(nx, nu, nz, b_f, b_g, b_h, b_q, b_r)};
  filter->state = b_x;
  filter->covariance = b_p;

  return filter;
}