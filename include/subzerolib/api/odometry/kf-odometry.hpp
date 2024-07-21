#include "subzerolib/api/filter/kalman-filter.hpp"
#include "subzerolib/api/odometry/gyro-odometry.hpp"

/// Requirements for the filter:
///
/// state vector of form [x, vx, ax, y, vy, ay, h, vh, ah]
/// measurement vector of form [x, vx, y, vy, h, vh]
// TODO: why cannot cast to private base type?
class KFOdometry : public GyroOdometry, private KalmanFilter {
public:
  virtual ~KFOdometry() {}

  /// @brief sets the heading of the odometry module
  /// @param heading the desired heading
  void set_heading(double heading) override;

  /// @brief sets the position of the odometry module
  /// @param x the desired position's x coordinate
  /// @param y the desired position's y coordinate
  void set_position(double x, double y) override;

  /// @brief get the current pose
  /// @returns the pose measurement
  pose_s get_pose() override;

  /// @brief get the current velocity
  /// @returns the velocity measurement
  pose_s get_vel() override;

  /// @brief trigger an update tick
  void update() override;

  /// @brief enable/disable the module
  /// @param bool desired state
  void set_enabled(bool) override;

  /// @brief check if the module is enabled
  /// @returns whether odometry is active
  bool is_enabled() override;

private:
  KFOdometry(std::shared_ptr<GyroOdometry> i_odom,
             std::shared_ptr<KalmanFilter> i_kf)
      : GyroOdometry(std::move(*i_odom)), KalmanFilter(std::move(*i_kf)) {}

public:
  class Builder : public GyroOdometry::Builder, public KalmanFilter::Builder {
  public:
    Builder(uint nx, uint nu, uint nz) : KalmanFilter::Builder(nx, nu, nz) {}
    std::shared_ptr<KFOdometry> build();
  };
};