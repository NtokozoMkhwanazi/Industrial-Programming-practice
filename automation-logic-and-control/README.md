# Automation Logic & Industrial Systems Simulator

> C++ implementations of the low-level control logic that drives a modern
> process plant — a closed-loop **PID temperature controller** and a
> safety-critical **finite state machine** for a conveyor routing system.
> Both are written from first principles to understand PLC/SCADA logic
> is built, and both ship with automated unit tests.

**Note** this project is still in early development stages but it wishes to achieve industrial standards.


* A **PID control loop** that regulates a furnace to an exact setpoint by
  manipulating a heater valve, complete with integrator anti-windup and
  output limiting.
* A **finite state machine (FSM)** that models a conveyor/material-handling
  line with strict, enumerated state transitions and a latched safety
  **lockout** that makes an unsafe restart impossible.



## Repository at a glance

```
automation-logic-and-control/
├── CMakeLists.txt              # Build: cmake -S . -B build && cmake --build build
├── README.md
├── docs/
│   └── DESIGN.md               # Tuning notes, state graph, safety rationale
├── include/                    # Public API (headers)
│   ├── pid/PIDController.hpp
│   └── fsm/ConveyorFSM.hpp
├── src/                        # Implementation
│   ├── pid/PIDController.cpp
│   └── fsm/ConveyorFSM.cpp
├── tests/                      # Zero-dependency unit tests ("Data Verification")
│   ├── test_framework.hpp
│   ├── test_main.cpp
│   ├── pid_tests.cpp
│   └── fsm_tests.cpp
└── examples/                   # Runnable demos
    ├── furnace_simulation.cpp   # ASCII text demo
    ├── furnace_dashboard.cpp    # SFML interactive dashboard (* Project Idea 1)
    ├── assets/                  # vendored phosphor icons + JetBrainsMono
    │   ├── icons/icon_*.png
    │   └── fonts/JetBrainsMono-SemiBold.ttf
    └── conveyor_demo.cpp
```

## Core Implementations

* **PID Process Control Loop** (`pid/`) — a deterministic C++ implementation of
  a feedback loop simulating real-time industrial temperature regulation.
  Features: proportional + integral + derivative terms, output clamping, and
  integrator anti-windup via *conditional integration* (the integrator is
  frozen while the valve is saturated in the direction of the error, which
  prevents wind-up-induced limit cycles during a long cold-to-hot soak).
* **Industrial Finite State Machine** (`fsm/`) — a safety-critical state
  machine architecture modelling conveyor material-handling states with
  automatic fault-handling routines and a hard lockout that rejects every
  command until the hazard is cleared and the operator resets.
* **Data Verification** — complete with automated unit tests (1054 checks)
  covering convergence, clamping, fault recovery, and lockout enforcement, to
  ensure logical stability under unexpected boundary inputs.

## Quick start

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build

./build/run_tests            # unit tests
./build/furnace_simulation   # Project Idea 1 demo (ASCII text)
./build/furnace_dashboard    # Project Idea 1 demo (SFML interactive dashboard, optional)
./build/conveyor_demo        # Project Idea 2 demo
```

## Quick start

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build

./build/run_tests            # unit tests (1054 checks)
./build/furnace_simulation   # Project Idea 1 demo (ASCII text, no display)
./build/furnace_dashboard    # Project Idea 1 demo (SFML interactive window)
./build/conveyor_demo        # Project Idea 2 demo
```

### Project Idea 1 — The Furnace PID Loop

A simulated meltshop furnace starts cold and a PID controller drives a heater
valve (%) to regulate it to **1600 C**:

```
step  time(s)  temp(C)   valve(%)  error(C)
   0     0.0       0.0    100.00   1600.00
  76     7.6    1575.1     92.93      30.74
Final temperature: 1600.00 C  (valve 80.00 %, error 0.00 C)
```

The window is an **interactive control dashboard** (SFML 2.6):

* **Thermometer gauge** — live furnace temperature vs. setpoint.
* **Valve bar** — live heater-output percentage (steady-state ≈ 80 % at 1600 C).
* **Scrolling plot** — temperature history vs. the setpoint line, with a
  colour-coded alarm lamp that ignites on over-temperature.
* **Tunable sliders** — real-time Kp / Ki / Kd / Setpoint / Ambient. Drag them
  and watch the loop respond instantly (increase Kp → faster but edgy; raise Ki
  → wind-up; add Kd → damping).
* **Control buttons** — Run / Pause / Stop / Reset, each rendered with a
  phosphor icon.

Default gains were swept against a no-limit-cycle criterion (low final error
**and** low late-window variance), landing on **Kp = 0.6, Ki = 0.3, Kd = 0.2**,
which regulates 0 → 1600 C cleanly with no steady-state oscillation.

### Running headless / for CI

The dashboard runs with no physical display under Xvfb (e.g. in CI or on a
headless server) and supports a `--auto` flag that runs a closed-loop regulation
to steady state and prints a summary, then exits:

```bash
cmake -S . -B build && cmake --build build   # furnace_dashboard is built automatically if SFML is found
xvfb-run -a ./build/furnace_dashboard --auto
# [auto] final temperature = 1600.00 C, setpoint = 1600 C, valve = 80.00 %, error = 0.00 C
```

`--auto` also writes `furnace_dashboard_screenshot.png` (the final frame) which
is handy for generating README art. If SFML is not installed, the target is
skipped automatically — the library and tests still build.

### Assets

The dashboard's modern look reuses assets already present in the sibling
**`3D GAME ENGINE`** project:

* **Phosphor icons** (regular, rasterised to 64×64 white-on-transparent) —
  play, pause, stop, reset, settings, thermometer, flame, setpoint, alarm,
  ambient, hot.
* **JetBrainsMono** (Nerd Fonts `JetBrainsMonoNLNerdFontMono-SemiBold.ttf`)
  as the UI typeface.

Both are vendored into `examples/assets/` so the repo is self-contained.

### Project Idea 2 — Structured Text / FSM Simulator

A scripted fault-and-recovery trace shows the safety rail in action:

```
event             -> state            result
MATERIAL_DETECTED -> MATERIAL_DETECTED accepted
START             -> PROCESSING        accepted
TEMP_HIGH         -> FAULT_OVERHEAT    accepted
RESET             -> LOCKOUT           accepted
START             -> LOCKOUT           REJECTED (safety lockout)
TEMP_NORMAL       -> LOCKOUT           accepted
RESET             -> IDLE              accepted
```

## State graph

```
                           +---------------------+
                           |        IDLE         |
                           +----------+----------+
                                      | MATERIAL_DETECTED
                                      v
                           +----------+----------+
                           |  MATERIAL_DETECTED   |
                           +----------+----------+
                                      | START
                                      v
                           +----------+----------+
                           |     PROCESSING       |
                           +----+------+----+-----+
                          FINISHED    TEMP_HIGH
                            |           v
                            |    +------+------+
                            |    | FAULT_OVERHEAT|
                            |    +------+------+
                            |    RESET |        (acknowledge)
                            |           v
                            |    +------+------+
                            v    |    LOCKOUT   |
                       +--------+---+   ^        |
                       | COMPLETE | RESET (clear cond.)
                       +----------+   +--------+
```

Recovery out of `LOCKOUT` is **two-step by design**: the operator must first
signal `TEMP_NORMAL` (the hazard is gone) and only then press `RESET`. In
`LOCKOUT` every other event is rejected, so the line cannot be started into
an unsafe condition.

## Use with caution

The simulations here are deterministic models, not live control code. They are
intended for learning, portfolio demonstration, and offline verification of
control logic. Do not connect them to physical equipment without an
independent safety assessment.

See [`docs/DESIGN.md`](docs/DESIGN.md) for tuning rationale and the full
transition table.
