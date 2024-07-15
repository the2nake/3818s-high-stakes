#pragma once

#include "subzerolib/api/filter/filter.hpp"

#include <memory>

template <typename state_vector_t, typename control_vector_t,
          typename measurement_vector_t>
class KalmanFilter
    : public Filter<state_vector_t, control_vector_t, measurement_vector_t> {
public:
  state_vector_t get_output() override;
  void predict(control_vector_t u) override;
  void update(measurement_vector_t z) override;

  KalmanFilter() {}
private:

  state_vector_t state;
  state_vector_t predicted_state;

public:
  class Builder {
  public:
    std::shared_ptr<KalmanFilter> build();
  };
};