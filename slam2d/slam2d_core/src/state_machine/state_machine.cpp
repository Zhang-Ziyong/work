#include "state_machine/state_machine.hpp"
#include <iostream>

namespace slam2d_core {
namespace state_machine {

static void goToErrorState(struct stateMachine *stateMachine,
                           struct event *const event);
static struct transition *getTransition(struct stateMachine *stateMachine,
                                        struct state *state,
                                        struct event *const event);

void stateM_init(struct stateMachine *fsm, struct state *initialState,
                 struct state *errorState) {
  if (!fsm)
    return;

  fsm->currentState = initialState;
  fsm->previousState = NULL;
  fsm->errorState = errorState;
}

int stateM_handleEvent(struct stateMachine *fsm, struct event *event) {
  if (!fsm || !event)
    return stateM_errArg;

  if (!fsm->currentState) {
    goToErrorState(fsm, event);
    return stateM_errorStateReached;
  }

  if (!fsm->currentState->numTransitions)
    return stateM_noStateChange;

  struct state *nextState = fsm->currentState;
  do {
    struct transition *transition = getTransition(fsm, nextState, event);

    if (!transition) {
      std::cout << "can't find transition " << std::endl;
      return -1;
    }

    if (!transition->nextState) {
      goToErrorState(fsm, event);
      return stateM_errorStateReached;
    }

    nextState = transition->nextState;

    while (nextState->entryState) nextState = nextState->entryState;

    if (nextState != fsm->currentState && fsm->currentState->exitAction)
      fsm->currentState->exitAction(fsm->currentState->data, event);

    if (transition->action)
      transition->action(fsm->currentState->data, event, nextState->data);

    if (nextState != fsm->currentState && nextState->entryAction)
      nextState->entryAction(nextState->data, event);

    fsm->previousState = fsm->currentState;
    fsm->currentState = nextState;

    if (fsm->currentState == fsm->previousState)
      return stateM_stateLoopSelf;

    if (fsm->currentState == fsm->errorState)
      return stateM_errorStateReached;

    if (!fsm->currentState->numTransitions)
      return stateM_finalStateReached;

    return stateM_stateChanged;
  } while (nextState);

  return stateM_noStateChange;
}

struct state *stateM_currentState(struct stateMachine *fsm) {
  if (!fsm)
    return NULL;

  return fsm->currentState;
}

struct state *stateM_previousState(struct stateMachine *fsm) {
  if (!fsm)
    return NULL;

  return fsm->previousState;
}

static void goToErrorState(struct stateMachine *fsm,
                           struct event *const event) {
  fsm->previousState = fsm->currentState;
  fsm->currentState = fsm->errorState;

  if (fsm->currentState && fsm->currentState->entryAction)
    fsm->currentState->entryAction(fsm->currentState->data, event);
}

static struct transition *getTransition(struct stateMachine *fsm,
                                        struct state *state,
                                        struct event *const event) {
  (void) fsm;
  size_t i;

  for (i = 0; i < state->numTransitions; ++i) {
    struct transition *t = &state->transitions[i];

    if (t->eventType == event->type) {
      if (!t->guard)
        return t;
      else if (t->guard(t->condition, event))
        return t;
    }
  }

  return NULL;
}

bool stateM_stopped(struct stateMachine *stateMachine) {
  if (!stateMachine)
    return true;

  return stateMachine->currentState->numTransitions == 0;
}

}  // namespace state_machine
}  // namespace slam2d_core
