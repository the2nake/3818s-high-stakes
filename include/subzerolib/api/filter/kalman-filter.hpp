#pragma once

#include "subzerolib/api/filter/filter.hpp"
#ifdef TARGET_V5
#include "pros/rtos.hpp"
#include "subzerolib/api/util/logging.hpp"
#endif
#include <memory>

class KalmanFilter : public Filter {
public:
  void predict(int delta_ms = 0) override;
  void predict(Eigen::VectorXd control_input, int delta_ms = 0) override;
  void update(Eigen::VectorXd measurement, int delta_ms = 0) override;

  Eigen::VectorXd get_state() override {
#ifdef TARGET_V5
    if (lock(5, "get_state")) {
#endif
      Eigen::VectorXd output = state;
#ifdef TARGET_V5
      unlock("get_state");
#endif
      return output;
#ifdef TARGET_V5
    }
    return {};
#endif
  }

  Eigen::MatrixXd get_covariance() override {
#ifdef TARGET_V5
    if (lock(5, "get_covariance")) {
#endif
      Eigen::MatrixXd output = covariance;
#ifdef TARGET_V5
      unlock("get_covariance");
#endif
      return output;
#ifdef TARGET_V5
    }
    return {};
#endif
  }

  Eigen::MatrixXd get_pred_state() { return next_state; }
  Eigen::MatrixXd get_pred_covariance() { return next_covariance; }

  void initialise(Eigen::VectorXd i_state,
                  Eigen::MatrixXd i_covariance) override {
#ifdef TARGET_V5
    if (lock(5, "initialise")) {
#endif
      for (int i = 0; i < i_state.rows() && i < state.rows(); ++i) {
        state(i) = i_state(i);
      }
      i_covariance.conservativeResize(nx, nx);
      covariance = i_covariance;
#ifdef TARGET_V5
      unlock("initialise");
    }
#endif
  }

  const uint nx;
  const uint nu;
  const uint nz;

private:
#ifdef TARGET_V5
  pros::Mutex mutex;

  bool lock(int dur = 5, const char *fnname = "") {
    bool empty = std::string{fnname}.empty();
    if (!mutex.take(dur)) {
      subzero::error(
          "[e]: %s%skf mutex failed to take", fnname, empty ? "" : "(): ");
      return false;
    }
    return true;
  }
  void unlock(const char *fnname = "") {
    bool empty = std::string{fnname}.empty();
    if (!mutex.give()) {
      subzero::error(
          "[e]: %s%s kf mutex failed to return", fnname, empty ? "" : "(): ");
    }
  }
#endif
  int n = 0;
  Eigen::VectorXd state;
  Eigen::MatrixXd covariance;

  Eigen::VectorXd next_state;
  Eigen::MatrixXd next_covariance;

  KalmanFilter(uint inx,
               uint inu,
               uint inz,
               Eigen::MatrixXd i_f,
               Eigen::MatrixXd i_g,
               Eigen::MatrixXd i_h,
               Eigen::MatrixXd i_q,
               Eigen::MatrixXd i_r)
      : nx(inx), nu(inu), nz(inz), state_transition_matrix(i_f),
        control_matrix(i_g), observation_matrix(i_h),
        process_noise_covariance(i_q), measurement_covariance(i_r) {}

  const Eigen::MatrixXd state_transition_matrix;
  const Eigen::MatrixXd control_matrix;
  const Eigen::MatrixXd observation_matrix;

  const Eigen::MatrixXd process_noise_covariance;
  const Eigen::MatrixXd measurement_covariance;

public:
  class Builder {
  public:
    /// \brief create a builder object for a kalman filter
    /// \param inx the size of the state vector
    /// \param inu the size of the control input vector
    /// \param inz the size of the measurement vector
    Builder(uint inx, uint inu, uint inz) : nx(inx), nu(inu), nz(inz) {}

    Builder &with_state_transition_matrix(Eigen::MatrixXd f);
    Builder &with_control_matrix(Eigen::MatrixXd g);
    Builder &with_observation_matrix(Eigen::MatrixXd h);

    Builder &with_process_noise_covariance(Eigen::MatrixXd q);
    Builder &with_measurement_covariance(Eigen::MatrixXd r);

    Builder &with_initial_state(Eigen::VectorXd x);
    Builder &with_initial_covariance(Eigen::MatrixXd p);

    std::shared_ptr<KalmanFilter> build();

  private:
    bool initialised(Eigen::MatrixXd &matrix) {
      return !std::isnan(matrix(0, 0));
    }

    bool initialised(Eigen::VectorXd &vec) { return !std::isnan(vec(0)); }

    const uint nx;
    const uint nu;
    const uint nz;

    Eigen::MatrixXd b_f{{std::nan("")}};
    Eigen::MatrixXd b_g{{std::nan("")}};
    Eigen::MatrixXd b_h{{std::nan("")}};

    Eigen::MatrixXd b_q{{std::nan("")}};
    Eigen::MatrixXd b_r{{std::nan("")}};

    Eigen::VectorXd b_x{{std::nan("")}};
    Eigen::MatrixXd b_p{{std::nan("")}};
  };

protected:
  KalmanFilter(KalmanFilter &&other)
      : KalmanFilter(other.nx,
                     other.nu,
                     other.nz,
                     other.state_transition_matrix,
                     other.control_matrix,
                     other.observation_matrix,
                     other.process_noise_covariance,
                     other.measurement_covariance) {
#ifdef TARGET_V5
    lock();
    // disable the other one
    other.lock(5, "KalmanFilter");
#endif
    state = other.state;
    covariance = other.covariance;
    next_state = other.next_state;
    next_covariance = other.next_covariance;
#ifdef TARGET_V5
    unlock();
#endif
  }
};
