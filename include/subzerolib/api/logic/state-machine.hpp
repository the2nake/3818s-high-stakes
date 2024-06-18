#pragma once
#include <map>
#include <string>

typedef void (*state_behaviour_fn_t)(void);
typedef bool (*state_condition_fn_t)(void);

class StateMachine {
public:
  struct State {
    State(std::string iname = "", state_behaviour_fn_t ibehaviour = nullptr,
          std::map<std::string, state_condition_fn_t> iexit_map = {})
        : name(iname), behaviour(ibehaviour), exit_map(iexit_map) {}

    std::string name;
    state_behaviour_fn_t behaviour;
    std::map<std::string, state_condition_fn_t> exit_map;
  };

  State get_curr_state() { return curr_state; }
  void exec_behaviour() {
    bool exit = false;
    do {
      exit = false;
      for (std::pair<const std::string, state_condition_fn_t> exit_pair :
           curr_state.exit_map) {
        if (exit_pair.second()) {
          curr_state = name_state_lookup.at(exit_pair.first);
        }
      }
    } while (exit);

    curr_state.behaviour();
  }

private:
  StateMachine() {}

  State curr_state;
  std::map<std::string, State> name_state_lookup;

public:
  class StateMachineBuilder {
  public:
    StateMachineBuilder &with_init(std::string state_name);
    StateMachineBuilder &with_state(State);
    StateMachine *build();

  private:
    std::string init_state = "";
    std::map<std::string, State> blookup;
  };
};