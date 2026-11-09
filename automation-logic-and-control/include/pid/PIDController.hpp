// PIDController.hpp
//
// A clean, deterministic C++17 implementation of a closed-loop PID
// (Proportional-Integral-Derivative) controller, the workhorse of
// industrial process control. This models the kind of loop that keeps a
// steel-mill furnace sitting at exactly 1600 C by adjusting a heater
// valve (%) based on the error between the temperature setpoint and the
// measured process variable.
//
// Design notes:
//   * Output limits + integrator anti-windup (conditional integration): when the
//     output is already at a rail AND the error would push it further into that
//     rail, the integrator is *frozen* instead of accumulating. This stops the
//     integrator from winding up during a long warm-up and then sticking there
//     (which would cause a limit cycle). Once the process variable crosses the
//     setpoint the integrator is allowed to unwind.
//   * Derivative-on-measurement style is avoided here for simplicity; the
//     derivative acts on the error which is fine for a simulator.
//   * `reset()` clears the integrator and derivative history so the loop
//     can be safely restarted after a fault or manual reset.

#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

namespace automation {

class PIDController {
public:
    // kp: proportional gain
    // ki: integral gain  (per second)
    // kd: derivative gain (seconds)
    // setpoint: target process variable (e.g. 1600.0 C)
    // outMin / outMax: clamp the final control output (valve %).
    PIDController(double kp, double ki, double kd,
                  double setpoint,
                  double outMin = -std::numeric_limits<double>::infinity(),
                  double outMax =  std::numeric_limits<double>::infinity());

    // Tuning / configuration.
    void setGains(double kp, double ki, double kd);
    void setSetpoint(double setpoint) { setpoint_ = setpoint; }
    void setOutputLimits(double outMin, double outMax);
    void setSampleTime(double seconds);   // seconds between calls

    // Clear integrator state and derivative history.
    void reset();

    // One control step. Give it the current measurement (process variable),
    // returns the new control output (e.g. heater valve %).
    double compute(double processVariable);

    // Read-backs (handy for tests and logging).
    double kp()        const { return kp_; }
    double ki()        const { return ki_; }
    double kd()        const { return kd_; }
    double setpoint()  const { return setpoint_; }
    double sampleTime() const { return sampleTime_; }
    double integral()  const { return integral_; }
    double lastError() const { return lastError_; }
    bool   firstCall() const { return firstCall_; }

private:
    double kp_;
    double ki_;
    double kd_;
    double setpoint_;
    double sampleTime_;
    double outMin_;
    double outMax_;
    double integral_;     // accumulated error * dt
    double lastError_;    // previous cycle's error
    bool   firstCall_;    // true until the first compute() runs
};

} // namespace automation
