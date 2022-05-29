#ifndef _STATE_MACHINE_H_
#define _STATE_MACHINE_H_

#include <stddef.h>
#include <stdbool.h>
#include <functional>

namespace slam2d_core {
namespace state_machine {

struct event {
  int type;
  void *data;
};

struct state;

struct transition {
  int eventType;
  void *condition;
  std::function<bool(void *condition, struct event *event)> guard;
  std::function<void(void *currentStateData, struct event *event,
                     void *newStateData)>
      action;
  struct state *nextState;
};

struct state {
  struct state *parentState;
  struct state *entryState;
  struct transition *transitions;
  size_t numTransitions;
  void *data;

  std::function<void(void *stateData, struct event *event)> entryAction;
  std::function<void(void *stateData, struct event *event)> exitAction;
};

struct stateMachine {
  struct state *currentState;
  struct state *previousState;
  struct state *errorState;
};

void stateM_init(struct stateMachine *stateMachine, struct state *initialState,
                 struct state *errorState);

enum stateM_handleEventRetVals {
  stateM_errArg = -2,
  stateM_errorStateReached,
  stateM_stateChanged,
  stateM_stateLoopSelf,
  stateM_noStateChange,
  stateM_finalStateReached,
};

int stateM_handleEvent(struct stateMachine *stateMachine, struct event *event);

struct state *stateM_currentState(struct stateMachine *stateMachine);

struct state *stateM_previousState(struct stateMachine *stateMachine);

bool stateM_stopped(struct stateMachine *stateMachine);

}  // namespace state_machine
}  // namespace slam2d_core
#endif  // STATEMACHINE_H
