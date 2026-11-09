// fsm_tests.cpp
// Unit tests for the conveyor FSM, focusing on the safety lockout behaviour.

#include "test_framework.hpp"

#include "fsm/ConveyorFSM.hpp"

void runFsmTests() {
    using automation::ConveyorFSM;
    using S  = ConveyorFSM::State;
    using E  = ConveyorFSM::Event;

    std::printf("== Conveyor FSM tests ==\n");

    {
        // 1. Happy path: IDLE -> MATERIAL_DETECTED -> PROCESSING -> COMPLETE -> IDLE.
        ConveyorFSM fsm;
        CHECK_EQ(fsm.currentState(), S::IDLE);

        CHECK(fsm.handle(E::MATERIAL_DETECTED) == true);
        CHECK_EQ(fsm.currentState(), S::MATERIAL_DETECTED);

        CHECK(fsm.handle(E::START) == true);
        CHECK_EQ(fsm.currentState(), S::PROCESSING);

        CHECK(fsm.handle(E::FINISHED) == true);
        CHECK_EQ(fsm.currentState(), S::COMPLETE);

        CHECK(fsm.handle(E::RESET) == true);
        CHECK_EQ(fsm.currentState(), S::IDLE);
        CHECK(fsm.isFaulted() == false);
    }

    {
        // 2. A start command issued while LOCKED OUT must be rejected.
        ConveyorFSM fsm;
        CHECK_EQ(fsm.currentState(), S::IDLE);

        // Trip the fault, acknowledge it (-> LOCKOUT), then try to start.
        CHECK(fsm.handle(E::MATERIAL_DETECTED) == true);
        CHECK(fsm.handle(E::START) == true);
        CHECK_EQ(fsm.currentState(), S::PROCESSING);

        CHECK(fsm.handle(E::TEMP_HIGH) == true);
        CHECK_EQ(fsm.currentState(), S::FAULT_OVERHEAT);

        CHECK(fsm.handle(E::RESET) == true);   // acknowledge -> LOCKOUT
        CHECK_EQ(fsm.currentState(), S::LOCKOUT);
        CHECK(fsm.isLockedOut() == true);

        // Safety rail: START is illegal while locked out.
        CHECK(fsm.handle(E::START) == false);
        CHECK_EQ(fsm.currentState(), S::LOCKOUT);

        // And so is FINISHED / MATERIAL_DETECTED.
        CHECK(fsm.handle(E::MATERIAL_DETECTED) == false);
        CHECK(fsm.handle(E::FINISHED) == false);
        CHECK_EQ(fsm.currentState(), S::LOCKOUT);
    }

    {
        // 3. Correct recovery sequence: TEMP_NORMAL then RESET to resume.
        ConveyorFSM fsm;
        // ... trip into LOCKOUT (same as above).
        CHECK(fsm.handle(E::MATERIAL_DETECTED) == true);
        CHECK(fsm.handle(E::START) == true);
        CHECK(fsm.handle(E::TEMP_HIGH) == true);
        CHECK_EQ(fsm.currentState(), S::FAULT_OVERHEAT);
        CHECK(fsm.handle(E::RESET) == true);
        CHECK_EQ(fsm.currentState(), S::LOCKOUT);

        // RESET before clearing the hazard -> stays locked.
        CHECK(fsm.handle(E::RESET) == true);
        CHECK_EQ(fsm.currentState(), S::LOCKOUT);

        // Clear the hazard, then RESET -> IDLE.
        CHECK(fsm.handle(E::TEMP_NORMAL) == true);
        CHECK_EQ(fsm.currentState(), S::LOCKOUT);
        CHECK(fsm.handle(E::RESET) == true);
        CHECK_EQ(fsm.currentState(), S::IDLE);
    }

    {
        // 4. Abort a cycle from MATERIAL_DETECTED back to IDLE.
        ConveyorFSM fsm;
        CHECK(fsm.handle(E::MATERIAL_DETECTED) == true);
        CHECK(fsm.handle(E::RESET) == true);
        CHECK_EQ(fsm.currentState(), S::IDLE);
    }

    {
        // 5. Audit trail records every accepted (state, event) pair.
        ConveyorFSM fsm;
        CHECK(fsm.handle(E::MATERIAL_DETECTED) == true);
        CHECK(fsm.handle(E::START) == true);
        CHECK(fsm.history().size() == 2);
    }

    {
        // 6. Human-readable names don't throw / return sane strings.
        CHECK(S::IDLE == S::IDLE);  // compile-time sanity of naming
        CHECK(ConveyorFSM::stateName(S::LOCKOUT) == "LOCKOUT");
        CHECK(ConveyorFSM::eventName(E::TEMP_HIGH) == "TEMP_HIGH");
    }
}
