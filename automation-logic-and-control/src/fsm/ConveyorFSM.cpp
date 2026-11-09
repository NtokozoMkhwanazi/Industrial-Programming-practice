// ConveyorFSM.cpp

#include "fsm/ConveyorFSM.hpp"

namespace automation {

ConveyorFSM::ConveyorFSM()
    : state_(State::IDLE), faultCleared_(false) {
}

bool ConveyorFSM::handle(Event e) {
    const auto next = transition(state_, e);
    if (!next) {
        // Rejected event: leave state untouched. This is the safety guard
        // rail that keeps a LOCKOUT machine from being started mid-fault.
        return false;
    }

    state_ = *next;
    if (e == Event::TEMP_NORMAL && state_ == State::LOCKOUT) {
        faultCleared_ = true;       // hazard now signalled clear
    }
    if (state_ == State::FAULT_OVERHEAT) {
        faultCleared_ = false;      // a fresh fault is never pre-cleared
    }
    history_.emplace_back(state_, e);
    return true;
}

std::optional<ConveyorFSM::State>
ConveyorFSM::transition(State s, Event e) {
    switch (s) {
        case State::IDLE:
            switch (e) {
                case Event::MATERIAL_DETECTED: return State::MATERIAL_DETECTED;
                case Event::RESET:             return State::IDLE;       // safe no-op
                default:                       return std::nullopt;
            }

        case State::MATERIAL_DETECTED:
            switch (e) {
                case Event::START:             return State::PROCESSING;
                case Event::RESET:             return State::IDLE;       // abort cycle
                case Event::TEMP_HIGH:         return State::FAULT_OVERHEAT;
                default:                       return std::nullopt;
            }

        case State::PROCESSING:
            switch (e) {
                case Event::FINISHED:          return State::COMPLETE;
                case Event::TEMP_HIGH:         return State::FAULT_OVERHEAT;
                default:                       return std::nullopt;
            }

        case State::COMPLETE:
            switch (e) {
                case Event::RESET:             return State::IDLE;       // ready next cycle
                case Event::TEMP_HIGH:         return State::FAULT_OVERHEAT;
                default:                       return std::nullopt;
            }

        case State::FAULT_OVERHEAT:
            // The fault must be acknowledged before the lockout latches.
            switch (e) {
                case Event::RESET:             return State::LOCKOUT;
                default:                       return std::nullopt;
            }

        case State::LOCKOUT:
            switch (e) {
                case Event::TEMP_NORMAL:       return State::LOCKOUT;  // clear flag, stay locked
                case Event::RESET:
                    if (faultCleared_) {
                        return State::IDLE;   // condition clear + acknowledged -> run
                    }
                    return State::LOCKOUT;    // still locked: hazard not cleared
                default:
                    return std::nullopt;      // every other event rejected mid-lockout
            }
    }

    return std::nullopt;
}

void ConveyorFSM::forceReset() {
    state_ = State::IDLE;
    faultCleared_ = false;
    // Note: a real panel would also latch a maintenance flag here.
}

std::string ConveyorFSM::stateName(State s) {
    switch (s) {
        case State::IDLE:              return "IDLE";
        case State::MATERIAL_DETECTED: return "MATERIAL_DETECTED";
        case State::PROCESSING:        return "PROCESSING";
        case State::COMPLETE:          return "COMPLETE";
        case State::FAULT_OVERHEAT:    return "FAULT_OVERHEAT";
        case State::LOCKOUT:           return "LOCKOUT";
    }
    return "UNKNOWN";
}

std::string ConveyorFSM::eventName(Event e) {
    switch (e) {
        case Event::MATERIAL_DETECTED: return "MATERIAL_DETECTED";
        case Event::START:             return "START";
        case Event::FINISHED:          return "FINISHED";
        case Event::TEMP_HIGH:         return "TEMP_HIGH";
        case Event::TEMP_NORMAL:       return "TEMP_NORMAL";
        case Event::RESET:             return "RESET";
    }
    return "UNKNOWN";
}

} // namespace automation
