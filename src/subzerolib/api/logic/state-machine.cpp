#include "subzerolib/api/logic/state-machine.hpp"

StateMachine::StateMachineBuilder &
StateMachine::StateMachineBuilder::with_init(state_e state_name) {
  init_state = state_name;

  return *this;
}

StateMachine::StateMachineBuilder &
StateMachine::StateMachineBuilder::with_state(StateMachine::state_data_s state_data) {
  if (blookup.find(state_data.state) == blookup.end()) {
    blookup.emplace(state_data.state, state_data);
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
    if (pair.first != pair.second.state) {
      return nullptr;
    }

    for (auto exit_pair : pair.second.exit_map) {
      if (blookup.find(exit_pair.first) == blookup.end()) {
        return nullptr;
      }
    }
  }

  obj->curr_state_data = blookup.at(init_state);
  obj->name_state_lookup = blookup;

  return obj;
}
