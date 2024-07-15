#pragma once

#include "subzerolib/api/filter/filter.hpp"

#include <memory>

class KalmanFilter : public Filter {
public:
  void predict(int delta_ms) override;
  void predict(int delta_ms, Eigen::VectorXd control_input) override;
  void update(int delta_ms, Eigen::VectorXd measurement) override;

  Eigen::VectorXd get_state() override { return state; }
  Eigen::MatrixXd get_covariance() override { return covariance; }

private:
  int n = 0;

  const uint nx;
  const uint nu;
  const uint nz;

  Eigen::VectorXd state;
  Eigen::MatrixXd covariance;

  Eigen::VectorXd next_state;
  Eigen::MatrixXd next_covariance;

  KalmanFilter(uint inx, uint inu, uint inz, Eigen::MatrixXd i_f,
               Eigen::MatrixXd i_g, Eigen::MatrixXd i_h, Eigen::MatrixXd i_q,
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
};