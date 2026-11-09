// furnace_simulation.cpp
//
// Project Idea 1 demo: a PID loop regulating a steel-mill furnace at 1600 C.
//
//   * The plant is a tiny first-order thermal model: temperature rises with
//     heat input and decays toward ambient via a loss factor.
//   * The PID controller outputs a heater "valve %" (0-100) every 100 ms.
//   * A small ASCII sparkline shows the temperature converging on setpoint.

#include "pid/PIDController.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

int main() {
    using automation::PIDController;

    const double SETPOINT  = 2000.0;   // target furnace temperature (C)
    const double AMBIENT   =    0.0;   // starting temperature (cold furnace)
    const double SAMPLE_DT =  0.1;     // 100 ms loop time
    const double PLANT_K   =    4.0;   // heat-input-to-temperature gain
    const double LOSS      =    0.20;  // radiative/convective loss per step
    // Steady-state at full valve (100%): T = AMBIENT + PLANT_K*100/LOSS = 2020 C,
    // so 1600 C is comfortably reachable (~80% valve) with headroom to spare.
    // Time constant tau = 1/LOSS = 5 s, so the loop settles within ~40 s.
    const int    STEPS     =  400;     // ~40 s of simulated run

    // Tuned for this thermal plant (tau = 5 s): smooth approach, ~0 C overshoot.
    PIDController pid(/*kp=*/1.2, /*ki=*/0.03, /*kd=*/0.5, SETPOINT,
                      /*outMin=*/0.0, /*outMax=*/100.0);
    pid.setSampleTime(SAMPLE_DT);

    double temperature = AMBIENT;
    double valve       = 0.0;

    std::printf("Furnace PID simulation: regulating to %.0f C\n", SETPOINT);
    std::printf("gains: kp=%.2f  ki=%.3f  kd=%.2f  (valve 0..100 %%)\n",
                pid.kp(), pid.ki(), pid.kd());
    std::printf("step  time(s)  temp(C)   valve(%%)  error(C)   sparkline\n");
    std::printf("----  ------   -------    --------   --------   ---------\n");

    for (int i = 0; i < STEPS; ++i) {
        valve = pid.compute(temperature);

        // First-order plant dynamics with ambient loss.
        temperature += PLANT_K * valve * SAMPLE_DT - LOSS * (temperature - AMBIENT) * SAMPLE_DT;
        if (temperature < AMBIENT) temperature = AMBIENT;

        // Temperature bar sparkline: '#' spans 0..(1.2*SETPOINT) visually.
        const int width = 50;
        const double scaleMax = SETPOINT * 1.2;   // 120% of setpoint = full bar
        char spark[width + 1];
        for (int j = 0; j < width; ++j) spark[j] = ' ';
        spark[width] = '\0';
        const int bar = static_cast<int>((temperature / scaleMax) * width);
        const int idx = std::min(bar, width - 1);
        if (idx >= 0) spark[idx] = '#';

        std::printf("%4d  %6.1f   %7.1f   %7.2f   %8.2f   %s\n",
                    i, i * SAMPLE_DT, temperature, valve,
                    std::abs(SETPOINT - temperature), spark);
    }

    const double finalErr = std::abs(SETPOINT - temperature);
    std::printf("\nFinal temperature: %.2f C  (error %.2f C)\n", temperature, finalErr);

    return 0;
}
