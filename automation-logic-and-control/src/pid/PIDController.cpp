// PIDController.cpp

#include "pid/PIDController.hpp"

namespace automation {

PIDController::PIDController(double kp, double ki, double kd,
                             double setpoint,
                             double outMin, double outMax)
    : kp_(kp), ki_(ki), kd_(kd),
      setpoint_(setpoint),
      sampleTime_(0.1),
      outMin_(outMin), outMax_(outMax),
      integral_(0.0), lastError_(0.0), firstCall_(true) {
}

void PIDController::setGains(double kp, double ki, double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PIDController::setOutputLimits(double outMin, double outMax) {
    outMin_ = outMin;
    outMax_ = outMax;
}

void PIDController::setSampleTime(double seconds) {
    sampleTime_ = seconds;
}

void PIDController::reset() {
    integral_   = 0.0;
    lastError_  = 0.0;
    firstCall_  = true;
}

double PIDController::compute(double processVariable) {
    const double error = setpoint_ - processVariable;
    const double dt    = sampleTime_;

    // --- Derivative term (skip the very first step to avoid a startup spike) ---
    double derivative;
    if (firstCall_) {
        derivative   = 0.0;
        firstCall_   = false;
    } else {
        derivative   = kd_ * (error - lastError_) / dt;
    }
    lastError_ = error;

    // Proportional + derivative part (independent of the integrator).
    const double propDer = kp_ * error + derivative;

    // Full output with the current integrator contribution.
    const double withIntegral = propDer + ki_ * integral_;

    // --- Anti-windup via "conditional integration" ---
    // When the output is already saturated AND the error would push it further
    // into the rail, we FREEZE the integrator (no accumulation). This is what
    // stops the integral from winding up to a huge value during a long warm-up
    // and then sticking there. Once the process variable crosses the setpoint
    // (error reverses sign) the integrator is allowed to unwind.
    if (withIntegral > outMax_) {
        if (error <= 0.0) integral_ += error * dt;   // unwinding
        return outMax_;
    } else if (withIntegral < outMin_) {
        if (error >= 0.0) integral_ += error * dt;   // unwinding
        return outMin_;
    }

    // Not saturated: integrate normally and recompute the output so the freshly
    // added integral term is reflected immediately.
    integral_ += error * dt;
    double output = propDer + ki_ * integral_;

    // --- Final output clamp (defensive) ---
    if (output > outMax_) {
        output = outMax_;
    } else if (output < outMin_) {
        output = outMin_;
    }

    return output;
}

} // namespace automation
