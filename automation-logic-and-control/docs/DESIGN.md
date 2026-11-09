# Design Notes — automation-logic-and-control

## 1. PID Controller

### Continuous-time form
The controller computes, once per sample period `dt`:

```
error   = setpoint - processVariable
integral += error * dt
derivative = (error - previousError) / dt
output   = Kp*error + Ki*integral + Kd*derivative
```

### Anti-windup (conditional integration — why it matters on a furnace)
A furnace heater valve saturates at 0–100 %. If the furnace is still far below
setpoint, the integrator keeps accumulating while the valve is already wide open;
when the temperature *finally* crosses setpoint the winded-up integrator keeps
the valve hard-open and you overshoot badly — and with a naive "clamp the
accumulator to the rail" scheme the integrator **sticks** at the rail and the
loop falls into a ±4 °C valve limit-cycle that *looks* regulated (error ≈ 0)
but never actually settles.

This implementation uses **conditional integration**: the output is computed
with the integrator at its current value, and *only* when the result is already
saturated **and** the error would push it further into that same rail do we
**freeze** the integrator (no accumulation). Once the process variable crosses
the setpoint and the error reverses sign, the integrator is free to unwind.
This prevents wind-up without ever pinning the integrator to the ceiling.

```
output = Kp*error + Ki*integral + Kd*derivative
if output > outMax and error >= 0:   # saturated high, pushing harder -> freeze
    output = outMax                 #   (do not integrate)
else if output < outMin and error <= 0:  # saturated low, pushing harder -> freeze
    output = outMin
else:                               # not saturated, or unwinding allowed
    integral += error * dt
    output = Kp*error + Ki*integral + Kd*derivative   # recompute with new integral
output = clamp(output, outMin, outMax)
```

### First-call derivative
On the very first `compute()` there is no previous error, so the derivative
term is forced to 0. Without this, the first output is an enormous spike that
can slam the valve to its limit. The unit tests explicitly assert this.

### Tuning the demo
Plant model (discrete, 100 ms loop):

```
T_{n+1} = T_n + K*valve*dt - LOSS*(T_n - AMBIENT)*dt
```

with `K = 4.0`, `LOSS = 0.20`, `AMBIENT = 0`, giving:
* steady-state ceiling at full valve: `AMBIENT + K*100/LOSS = 2020 C` (so
  1600 C is reachable at roughly 80 % valve, with headroom);
* time constant `tau = 1/LOSS = 5 s`, so the loop settles within ~40 s.

Gains `Kp=1.2, Ki=0.03, Kd=0.5` were an earlier first guess, but on this plant
they settle into a **stable 2-cycle limit oscillation** (valve flips between
≈59 % and 100 %; the integrator pins itself to the ceiling at `Ki*integral≈100`)
that the naive "final error" metric falsely accepted. The gain sweep was
re-run with a *smoothness* criterion — low final error **and** low late-window
variance (rejecting limit cycles) — and converged to
**`Kp=0.6, Ki=0.3, Kd=0.2`**, which regulates 0 → 1600 °C with effectively zero
overshoot (final error ≈ 0.00 °C) and a steady-state valve of ≈ 80 %.

## 2. Conveyor FSM

### Safety rationale for the lockout
A real conveyor does **not** let you hit "Start" while a fault lamp is lit.
The state machine enforces this structurally: while `LOCKOUT`, `handle()`
returns `false` for every event except `TEMP_NORMAL` (clear condition) and
`RESET` (acknowledge). The `RESET` is *only* honoured when the condition has
already been cleared, which means an operator cannot restart a hot machine
blindly.

### Transition table (excerpt)

| State             | Event               | Next state            | Accepted? |
|-------------------|---------------------|-------------------------|-----------|
| IDLE              | MATERIAL_DETECTED   | MATERIAL_DETECTED     | yes       |
| IDLE              | RESET               | IDLE                    | yes       |
| MATERIAL_DETECTED | START               | PROCESSING            | yes       |
| MATERIAL_DETECTED | RESET               | IDLE                    | yes       |
| PROCESSING        | FINISHED            | COMPLETE              | yes       |
| PROCESSING        | TEMP_HIGH           | FAULT_OVERHEAT        | yes       |
| FAULT_OVERHEAT    | RESET               | LOCKOUT               | yes       |
| LOCKOUT           | TEMP_NORMAL         | LOCKOUT (clears flag) | yes       |
| LOCKOUT           | RESET (hazard clear)| IDLE                  | yes       |
| LOCKOUT           | any other           | LOCKOUT               | **no**    |

### Why `std::optional<State>`
`transition()` returns `std::optional<State>`: `nullopt` means "this event is
illegal here", which is far safer than returning a sentinel state or silently
staying put. The unit tests assert that rejected events leave the state
unchanged.

## 3. Testing approach

Tests live in `tests/` and use a tiny self-contained header
(`test_framework.hpp`) rather than an external framework, so the repo
verifies cleanly in an air-gapped industrial environment. The harness is the
only thing that would change if you later swap in Catch2 or GoogleTest — the
test bodies (`CHECK`, `CHECK_EQ`) stay the same.

Run with:
```
cmake -S . -B build && cmake --build build && ./build/run_tests
```
