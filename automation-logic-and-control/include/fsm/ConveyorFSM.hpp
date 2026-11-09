// ConveyorFSM.hpp
//
// A Finite State Machine (FSM) modelling the control logic of a conveyor
// / routing station. Industrial PLCs rely heavily on strict, enumerated
// state transitions with explicit fault handling and a safety "lockout"
// that prevents the line from restarting while a hazard is present.
//
// The state graph below implements a realistic four-step safety dance:
//
//     active run  -->  FAULT_OVERHEAT  -->  LOCKOUT  -->  IDLE  -->  run
//        (TEMP_HIGH)    (acknowledge)    (clear cond)  (reset)
//
// Once a temperature fault latches the system into LOCKOUT, no "start"
// event is accepted until an operator has (a) signalled the hazard is
// clear (TEMP_NORMAL) and then (b) pressed RESET. That is the
// "safety lockout override" the brief calls out.

#pragma once

#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace automation {

class ConveyorFSM {
public:
    // States the conveyor can occupy.
    enum class State {
        IDLE,
        MATERIAL_DETECTED,
        PROCESSING,
        COMPLETE,
        FAULT_OVERHEAT,
        LOCKOUT
    };

    // External events / sensor signals fed to the controller.
    enum class Event {
        MATERIAL_DETECTED,   // photo-eye sees stock on the belt
        START,                // operator / upstream start command
        FINISHED,             // process step finished normally
        TEMP_HIGH,            // over-temperature safety alarm
        TEMP_NORMAL,          // temperature returned to safe band
        RESET                 // manual reset push-button
    };

    ConveyorFSM();

    // Current state (read-only view of the controller).
    State currentState() const { return state_; }

    // True when sitting in a faulted / locked state.
    bool isFaulted() const {
        return state_ == State::FAULT_OVERHEAT || state_ == State::LOCKOUT;
    }
    bool isLockedOut() const { return state_ == State::LOCKOUT; }

    // Feed an event to the controller.
    // Returns true  -> event accepted, a transition occurred.
    // Returns false -> event rejected (illegal in current state, e.g. a
    //                  START while LOCKOUT). State is unchanged.
    bool handle(Event e);

    // Force the machine back to IDLE (bypasses the lockout). Use only for
    // commissioning/tests; real recovery goes through handle(RESET).
    void forceReset();

    // Human-readable name for logging / dashboard display.
    static std::string stateName(State s);
    static std::string eventName(Event e);

    // Audit trail: every accepted (state, event) pair, for traceability.
    const std::vector<std::pair<State, Event>>& history() const { return history_; }

private:
    // Pure transition logic. Returns the new state, or std::nullopt if the
    // event is rejected in the current state.
    std::optional<State> transition(State s, Event e);

    State state_;
    bool  faultCleared_;   // set true when TEMP_NORMAL seen in LOCKOUT
    std::vector<std::pair<State, Event>> history_;
};

} // namespace automation
