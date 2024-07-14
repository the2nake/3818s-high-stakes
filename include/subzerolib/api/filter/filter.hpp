#pragma once

template <typename state_vector_t, typename control_vector_t,
          typename measurement_vector_t>
class Filter {
public:
  virtual state_vector_t get_output() = 0;
  virtual void update(measurement_vector_t z) = 0;
  virtual void predict(control_vector_t u) = 0;

protected:
  Filter() {}
};