#include "subzerolib/api/odometry/gyro-odometry.hpp"
#include "subzerolib/api/filter/filter.hpp"
#include "subzerolib/api/util/math.hpp"

void GyroOdometry::set_heading(double i_h) {
  lock();
  this->pose.h = i_h;
  if (filter_config != filter_config_e::none) {
    auto state = filter->get_state();
    state(0) = i_h;
    auto cov = filter->get_covariance();
    filter->initialise(state, cov);
  }
  unlock();
}

void GyroOdometry::set_position(double i_x, double i_y) {
  lock();
  this->pose.x = i_x;
  this->pose.y = i_y;
  if (filter_config != filter_config_e::none) {
    auto state = filter->get_state();
    state(1) = i_x;
    state(2) = i_y;
    auto cov = filter->get_covariance();
    filter->initialise(state, cov);
  }
  unlock();
}

void GyroOdometry::update() {
  auto now = pros::millis();
  double dt = (now - prev_timestamp) * 0.001;
  prev_timestamp = now;

  double dh = shorter_turn(prev_heading, gyro->heading());
  if (std::isnan(dh)) {
    dh = 0;
  }

  std::vector<double> x_enc_raws(x_encs.size());
  std::vector<double> y_enc_raws(y_encs.size());
  bool skip = false;

  for (int i = 0; i < x_encs.size(); ++i) {
    auto curr = x_encs[i].first->get_deg();
    double d_raw =
        x_encs[i].second.travel_per_deg * (curr - prev_x_enc_vals[i]);
    prev_x_enc_vals[i] = curr;
    if (std::isnan(d_raw)) {
      skip = true;
    }
    x_enc_raws[i] = d_raw;
  }
  for (int i = 0; i < y_encs.size(); ++i) {
    auto curr = y_encs[i].first->get_deg();
    double d_raw =
        y_encs[i].second.travel_per_deg * (curr - prev_y_enc_vals[i]);
    prev_y_enc_vals[i] = curr;
    if (std::isnan(d_raw)) {
      skip = true;
    }
    y_enc_raws[i] = d_raw;
  }
  if (skip) {
    return;
  }

  if (filter_config == filter_config_e::raw) {
    Eigen::VectorXd measurements;
    measurements.resize(1 + x_enc_raws.size() + y_enc_raws.size());
    measurements(0) = dh;
    for (int i = 0; i < x_enc_raws.size(); ++i) {
      measurements(1 + i) = x_enc_raws[i];
    }
    for (int i = 0; i < y_enc_raws.size(); ++i) {
      measurements(1 + x_enc_raws.size() + i) = y_enc_raws[i];
    }
    filter->predict(1000.0 * dt);
    filter->update(1000.0 * dt, measurements);
    update_pose_from_filter();
  }

  double x_impact_lx = 0.0, x_impact_ly = 0.0, y_impact_lx = 0.0,
         y_impact_ly = 0.0;
  // for reference: maximum rotation = 2 deg / 10 ms
  bool is_low_turn = std::abs(dh) < 0.01;

  for (int i = 0; i < x_enc_raws.size(); ++i) {
    if (is_low_turn) {
      x_impact_lx += x_enc_raws[i];
    } else {
      double tmp = (x_enc_raws[i] / in_rad(dh) - x_encs[i].second.offset);
      x_impact_lx += std::sin(in_rad(dh)) * tmp;
      x_impact_ly += (std::cos(in_rad(dh)) - 1) * tmp;
    }
  }
  for (int i = 0; i < y_enc_raws.size(); ++i) {
    if (is_low_turn) {
      y_impact_ly += y_enc_raws[i];
    } else {
      double tmp = (y_enc_raws[i] / in_rad(dh) + y_encs[i].second.offset);
      y_impact_lx += (1 - std::cos(in_rad(dh))) * tmp;
      y_impact_ly += std::sin(in_rad(dh)) * tmp;
    }
  }
  if (x_encs.size() > 0) {
    x_impact_lx /= x_encs.size();
    x_impact_ly /= x_encs.size();
  }
  if (y_encs.size() > 0) {
    y_impact_lx /= y_encs.size();
    y_impact_ly /= y_encs.size();
  }

  auto dx_l = x_impact_lx + y_impact_lx;
  auto dy_l = x_impact_ly + y_impact_ly;
  auto dx_g = dx_l * std::cos(in_rad(prev_heading)) +
              dy_l * std::sin(in_rad(prev_heading));
  auto dy_g = -dx_l * std::sin(in_rad(prev_heading)) +
              dy_l * std::cos(in_rad(prev_heading));
  prev_heading = pose.heading();

  if (filter_config == filter_config_e::none) {
    if (enabled) {
      lock();
      pose.x += dx_g;
      pose.y += dy_g;
      pose.h = mod(pose.h + dh, 360.0);
      unlock();
    }
  } else if (filter_config != filter_config_e::raw) {
    filter->predict(1000.0 * dt); // TODO: fix slight lag error with high [dt]
    if (filter_config == filter_config_e::local) {
      filter->update(1000.0 * dt,
                     Eigen::Vector<double, 3>{
                         {dh / dt, dx_l / dt, dy_l / dt}
      });
    } else if (filter_config == filter_config_e::global) {
      filter->update(1000.0 * dt,
                     Eigen::Vector<double, 3>{
                         {dh / dt, dx_g / dt, dy_g / dt}
      });
    }
    update_pose_from_filter();
  }
}

void GyroOdometry::update_pose_from_filter() {
  auto state = filter->get_state();
  if (enabled) {
    lock();
    pose.h = mod(state(0), 360.0);
    pose.x = state(1);
    pose.y = state(2);
    unlock();
  }
}

GyroOdometry::Builder &
GyroOdometry::Builder::with_gyro(std::shared_ptr<AbstractGyro> igyro) {
  gyro = std::move(igyro);
  return *this;
}

GyroOdometry::Builder &
GyroOdometry::Builder::with_x_enc(std::shared_ptr<AbstractEncoder> encoder,
                                  encoder_conf_s conf) {
  x_encs.emplace_back(std::move(encoder), conf);
  return *this;
}

GyroOdometry::Builder &
GyroOdometry::Builder::with_y_enc(std::shared_ptr<AbstractEncoder> encoder,
                                  encoder_conf_s conf) {
  y_encs.emplace_back(std::move(encoder), conf);
  return *this;
}

GyroOdometry::Builder &
GyroOdometry::Builder::with_filter(std::unique_ptr<Filter> i_filter,
                                   filter_config_e i_config) {
  if (i_filter != nullptr) {
    this->filter = std::move(i_filter);
  }
  this->config = i_config;
  return *this;
}

std::shared_ptr<GyroOdometry> GyroOdometry::Builder::build() {
  if (gyro == nullptr) {
    return nullptr;
  }
  for (auto pair : x_encs) {
    if (pair.first == nullptr) {
      return nullptr;
    }
  }
  for (auto pair : y_encs) {
    if (pair.first == nullptr) {
      return nullptr;
    }
  }
  if (x_encs.size() == 0 || y_encs.size() == 0) {
    return nullptr;
  }

  std::shared_ptr<GyroOdometry> odom(new GyroOdometry());

  odom->prev_timestamp = pros::millis();
  odom->gyro = gyro;
  odom->x_encs = x_encs;
  odom->y_encs = y_encs;

  odom->prev_x_enc_vals = std::vector<double>(x_encs.size(), std::nan(""));
  odom->prev_y_enc_vals = std::vector<double>(y_encs.size(), std::nan(""));
  odom->pose = pose_s{0, 0, 0};

  if (filter != nullptr) {
    odom->filter = std::move(filter);
    odom->filter_config = config;
  }

  return odom;
}
