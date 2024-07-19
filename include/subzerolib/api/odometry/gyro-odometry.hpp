#pragma once

#include "subzerolib/api/odometry/odometry.hpp"
#include "subzerolib/api/sensors/abstract-encoder.hpp"
#include "subzerolib/api/sensors/abstract-gyro.hpp"
#include "subzerolib/api/util/logging.hpp"

#include "pros/rtos.hpp"
#include <memory>

class GyroOdometry : public Odometry {
public:
  enum class filter_config_e {
    none,
    raw,   // direct imu, encoder measurements
    local, // changes calculated to local
    global // calculated global movements
  };

  /// @brief sets the heading of the odometry module
  /// @param heading the desired heading
  void set_heading(double i_h) override;

  /// @brief sets the position of the odometry module
  /// @param x the desired position's x coordinate
  /// @param y the desired position's y coordinate
  void set_position(double i_x, double i_y) override;

  /// @brief get the current pose
  /// @returns the pose measurement
  pose_s get_pose() override {
    lock();
    auto temp = pose;
    unlock();
    return temp;
  }

  /// @brief get the current velocity
  /// @returns the velocity measurement
  point_s get_vel() override { return point_s{0, 0}; }

  /// @brief trigger an update tick
  void update() override;

  /// @brief enable/disable the module
  /// @param bool desired state
  void set_enabled(bool v) override { enabled = v; }

  /// @brief check if the module is enabled
  /// @returns whether odometry is active
  bool is_enabled() override { return enabled.load(); }

private:
  pros::Mutex mutex;
  std::atomic<bool> enabled = true;

  std::shared_ptr<AbstractGyro> gyro;
  std::vector<std::pair<std::shared_ptr<AbstractEncoder>, encoder_conf_s>>
      x_encs;
  std::vector<std::pair<std::shared_ptr<AbstractEncoder>, encoder_conf_s>>
      y_encs;

  uint32_t prev_timestamp = 0;
  double prev_heading = 0.0;
  std::vector<double> prev_x_enc_vals;
  std::vector<double> prev_y_enc_vals;

  pose_s pose{0, 0, 0};

  GyroOdometry() {}
  void lock() {
    if (!mutex.take(5)) {
      subzero::error("[e]: gyro odom mutex failed to take");
    }
  }
  void unlock() {
    if (!mutex.give()) {
      subzero::error("[e]: gyro odom mutex failed to return");
    }
  }

  void update_pose_from_filter();

public:
  class Builder {
  public:
    Builder() {}
    Builder(Builder &&other) {
      gyro = other.gyro;
      x_encs = other.x_encs;
      y_encs = other.y_encs;
    }

    Builder &with_gyro(std::shared_ptr<AbstractGyro> igyro);

    Builder &with_x_enc(std::shared_ptr<AbstractEncoder> encoder,
                        encoder_conf_s conf);

    Builder &with_y_enc(std::shared_ptr<AbstractEncoder> encoder,
                        encoder_conf_s conf);

    std::shared_ptr<GyroOdometry> build();

  private:
    std::shared_ptr<AbstractGyro> gyro;
    std::vector<std::pair<std::shared_ptr<AbstractEncoder>, encoder_conf_s>>
        x_encs;
    std::vector<std::pair<std::shared_ptr<AbstractEncoder>, encoder_conf_s>>
        y_encs;
  };
};