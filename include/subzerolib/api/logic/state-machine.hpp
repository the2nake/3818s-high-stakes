#pragma once
#include <functional>
#include <map>
#include <string>

class StateMachine {
public:
  struct State {
    State(std::string iname = "", std::function<void()> ibehaviour = nullptr,
          std::map<std::string, std::function<bool()>> iexit_map = {})
        : name(iname), behaviour(ibehaviour), exit_map(iexit_map) {}

    std::string name;
    std::function<void()> behaviour;
    std::map<std::string, std::function<bool()>> exit_map;
  };

  State get_curr_state() { return curr_state; }
  void exec_behaviour() {
    bool exit = false;
    do {
      exit = false;
      for (std::pair<const std::string, std::function<bool()>> exit_pair :
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