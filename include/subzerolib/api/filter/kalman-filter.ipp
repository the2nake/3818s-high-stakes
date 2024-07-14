#include "subzerolib/api/filter/kalman-filter.hpp"
#include "eigen/Dense"

template <typename state_vector_t, typename control_vector_t,
          typename measurement_vector_t>
state_vector_t KalmanFilter<state_vector_t, control_vector_t,
                            measurement_vector_t>::get_output() {
  return state;
}

template <typename state_vector_t, typename control_vector_t,
          typename measurement_vector_t>
void KalmanFilter<state_vector_t, control_vector_t,
                  measurement_vector_t>::predict(control_vector_t u) {}

template <typename state_vector_t, typename control_vector_t,
          typename measurement_vector_t>
void KalmanFilter<state_vector_t, control_vector_t,
                  measurement_vector_t>::update(measurement_vector_t z) {}
