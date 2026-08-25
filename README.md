# motorDynamics

Pure-C electric-motor plant models and control foundations for Antshiv
Robotics. A brushed DC model establishes the simplest electrical/mechanical
reference. The first BLDC model exposes three phase currents, trapezoidal
back-EMF, electrical angle, electromagnetic torque, and averaged six-step
inverter voltage vectors.

The portable six-step controller adds explicit phase commands, alignment,
forced startup, observed commutation, bounded voltage/current operation, and
latched faults. Hardware-specific PWM, ADC, SPI, and timer code remains outside
the numerical library so the same contract can target KV31F, KV58, and i.MX
RT1176 adapters.

## Accepted model

The first model integrates current, shaft speed, and shaft angle:

```text
L di/dt = V - R i - K_e omega
J domega/dt = K_t i - b omega - tau_load
dtheta/dt = omega
```

All public values use SI units. Checked calls reject invalid configurations,
clear derivative outputs on failure, and leave state unchanged when an RK4
stage fails.

The BLDC plant evaluates a wye-connected three-phase machine:

```text
L di_p/dt = v_p - v_neutral - R i_p - e_p(theta_e, omega)
theta_e = pole_pairs * theta_m
J domega/dt = torque_e(i_a, i_b, i_c, theta_e) - b omega - torque_load
```

The neutral potential is solved each evaluation so a valid zero-sum phase
current remains zero-sum. Inverter voltage generation is separate from the
machine equations.

## Build and validate

```bash
cmake -S . -B build
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

The embedded portability gate can be run with:

```bash
sh scripts/check_cortex_m4.sh
```

The native suite checks derivatives, equilibrium, transient convergence,
power observations, and fail-closed behavior. When `uv` is available, the
same transient is compared with an independently integrated SciPy oracle.

## Repository boundary

- `motorDynamics` owns electric-machine, inverter, commutation, motor-control,
  and parameter-identification mathematics.
- `rotorDynamics` owns aerodynamic shaft load, thrust, and reaction torque.
- `dynamic_models` composes those libraries into a vehicle plant.
- Hardware-specific PWM, ADC, timers, and gate-driver code belongs in the
  flight-controller integration layer.

## Planned progression

1. Brushed DC motor parameter identification and current/voltage limits.
2. Add BEMF blanking, zero-cross qualification, and commutation advance as a
   separately tested sensorless observer.
3. Replace the accepted averaged six-step vectors with switching/dead-time and
   current-limit models.
4. Clarke/Park transforms and PMSM field-oriented control.
5. Hall and encoder observers.
6. Coupled motor-propeller and battery-bus fixtures.
