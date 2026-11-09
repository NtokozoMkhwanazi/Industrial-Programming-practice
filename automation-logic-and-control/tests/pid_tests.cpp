// pid_tests.cpp
// Unit tests for the PID controller. Run via tests/test_main.cpp.

#include "test_framework.hpp"

#include "pid/PIDController.hpp"

#include <cmath>

void runPidTests() {
    using automation::PIDController;

    std::printf("== PID Controller tests ==\n");

    {
        // 1. First compute() must not throw a derivative spike.
        PIDController pid(2.0, 0.5, 0.01, 1600.0);
        pid.setSampleTime(0.1);
        const double out = pid.compute(20.0);   // huge initial error
        CHECK(std::isfinite(out));
        CHECK(pid.firstCall() == false);  // derivative guard flipped after one call
    }

    {
        // 2. Proportional-only loop: output should equal Kp * error at t0
        //    (integral and derivative contributions start at zero).
        PIDController pid(3.0, 0.0, 0.0, 1600.0, 0.0, 100.0);
        pid.setSampleTime(0.1);
        const double out = pid.compute(1500.0);  // error = 100
        CHECK(std::abs(out - 100.0) < 1e-9);
    }

    {
        // 3. Output clamping: a massive error must saturate at outMax.
        PIDController pid(10.0, 0.0, 0.0, 1600.0, 0.0, 100.0);
        const double out = pid.compute(0.0);    // error = 1600 -> 16000 clamped
        CHECK_EQ(out, 100.0);
    }

    {
        // 4. Integrator anti-windup: long run far below setpoint must hold
        //    the output at the ceiling, not drift upward forever.
        PIDController pid(1.0, 5.0, 0.0, 1600.0, 0.0, 100.0);
        pid.setSampleTime(0.1);
        for (int i = 0; i < 1000; ++i) {
            const double out = pid.compute(20.0);
            CHECK_EQ(out, 100.0);  // pinned at ceiling every step
        }
    }

    {
        // 5. Reset clears the integrator: run in a *non-saturated* region so the
        //    integrator actually accumulates, then confirm reset zeroes it.
        PIDController pid(0.1, 0.5, 0.0, 100.0, -50.0, 50.0);
        pid.setSampleTime(0.1);
        pid.compute(80.0);           // error = 20  -> well inside the rails
        pid.compute(80.0);
        const double before = pid.integral();
        CHECK(std::abs(before) > 0.0);   // integrator did accumulate
        pid.reset();
        CHECK_EQ(pid.integral(), 0.0);
        CHECK_EQ(pid.lastError(), 0.0);
        CHECK(pid.firstCall() == true);
    }

    {
        // 6. A pure integral (Ki) controller on a stable plant should drive
        //    the steady-state error toward zero over time.
        PIDController pid(0.0, 0.3, 0.0, 1600.0, 0.0, 100.0);
        pid.setSampleTime(0.1);
        double pv = 20.0;            // furnace starts cold
        double valve = 0.0;
        const double k = 2.0;        // heat-input-to-temperature gain
        const double LOSS = 0.10;    // radiative/convective loss per step
        // Steady-state ceiling at valve=100 is k*100/LOSS = 2000 C, so
        // 1600 C is reachable (valve ~80%) and the plant is stable.
        for (int i = 0; i < 5000; ++i) {
            valve = pid.compute(pv);
            pv += k * valve * 0.1 - LOSS * (pv - 20.0) * 0.1;  // heat in minus loss
        }
        // A pure integrator drives steady-state error to ~0 over time.
        const double error = std::abs(1600.0 - pv);
        CHECK(error < 50.0);   // within ~50 C of target
    }
}
