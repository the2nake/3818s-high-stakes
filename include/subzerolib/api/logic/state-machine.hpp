#pragma once
#include <functional>
#include <map>

class StateMachine {
public:
  enum class state_e {
    null,
  };

  struct state_data_s {
    state_data_s(state_e icurr = state_e::null,
                 std::function<void()> ibehaviour = nullptr,
                 std::map<state_e, std::function<bool()>> iexit_map = {})
        : state(icurr), behaviour(ibehaviour), exit_map(iexit_map) {}

    state_e state;
    std::function<void()> behaviour;
    std::map<state_e, std::function<bool()>> exit_map;
  };

  state_data_s get_curr_state_data() { return curr_state_data; }
  void exec_behaviour() {
    bool exit = false;
    do {
      exit = true;
      for (auto &exit_pair : curr_state_data.exit_map) {
        if (exit_pair.second()) {
          curr_state_data = name_state_lookup.at(exit_pair.first);
          exit = false;
        }
      }
    } while (exit);

    curr_state_data.behaviour();
  }

private:
  StateMachine() {}

  state_data_s curr_state_data;
  std::map<state_e, state_data_s> name_state_lookup;

public:
  class StateMachineBuilder {
  public:
    StateMachineBuilder &with_init(state_e state_name);
    StateMachineBuilder &with_state(state_data_s);
    StateMachine *build();

  private:
    state_e init_state = state_e::null;
    std::map<state_e, state_data_s> blookup;
  };
};