#include "subzerolib/api/logic/state-machine.hpp"

StateMachine::StateMachineBuilder &
StateMachine::StateMachineBuilder::with_init(std::string state_name) {
  init_state = state_name;

  return *this;
}

StateMachine::StateMachineBuilder &
StateMachine::StateMachineBuilder::with_state(StateMachine::State state) {
  if (blookup.find(state.name) == blookup.end()) {
    blookup.emplace(state.name, state);
  }

  return *this;
}

StateMachine *StateMachine::StateMachineBuilder::build() {
  auto obj = new StateMachine();

  if (blookup.find(init_state) == blookup.end()) {
    return nullptr;
  }

  // validate all states
  for (auto pair : blookup) {
    for (auto exit_pair : pair.second.exit_map) {
      if (blookup.find(exit_pair.first) == blookup.end()) {
        return nullptr;
      }
    }
  }

  obj->curr_state = blookup.at(init_state);
  obj->name_state_lookup = blookup;

  return obj;
}
