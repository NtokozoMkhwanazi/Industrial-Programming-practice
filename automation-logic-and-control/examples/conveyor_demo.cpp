// conveyor_demo.cpp
//
// Project Idea 2 demo: drives the conveyor FSM through a scripted sequence
// of events, including a temperature fault and the required recovery
// (acknowledge -> LOCKOUT -> clear hazard -> RESET), printing each
// accepted/rejected transition so the safety lockout is visible.

#include "fsm/ConveyorFSM.hpp"

#include <cstdio>
#include <vector>

int main() {
    using ConveyorFSM = automation::ConveyorFSM;
    using E = ConveyorFSM::Event;

    ConveyorFSM fsm;

    // A scripted event sequence: normal cycle, then an over-temperature fault,
    // then the full safe-recovery sequence.
    const std::vector<E> script = {
        E::MATERIAL_DETECTED,  // IDLE -> MATERIAL_DETECTED
        E::START,              //            -> PROCESSING
        E::TEMP_HIGH,          //            -> FAULT_OVERHEAT (trip)
        E::RESET,              //            -> LOCKOUT (acknowledge fault)
        E::START,              //            -> rejected (safety rail!)
        E::RESET,              //            -> stays LOCKOUT (hazard not cleared)
        E::TEMP_NORMAL,        //            -> hazard cleared
        E::RESET,              //            -> IDLE (resume)
        E::MATERIAL_DETECTED,  //            -> MATERIAL_DETECTED
        E::START,              //            -> PROCESSING
        E::FINISHED,           //            -> COMPLETE
        E::RESET,              //            -> IDLE
    };

    std::printf("Conveyor FSM live trace:\n");
    std::printf("%-18s -> %-18s  %s\n", "event", "state", "result");
    std::printf("-------------------------------------------------------\n");

    for (const auto e : script) {
        const bool ok = fsm.handle(e);
        std::printf("%-18s -> %-18s  %s\n",
                    ConveyorFSM::eventName(e).c_str(),
                    ConveyorFSM::stateName(fsm.currentState()).c_str(),
                    ok ? "accepted" : "REJECTED (safety lockout)");
    }

    std::printf("\nFinal state: %s\n", ConveyorFSM::stateName(fsm.currentState()).c_str());
    std::printf("Audit trail entries: %zu\n", fsm.history().size());

    return 0;
}
