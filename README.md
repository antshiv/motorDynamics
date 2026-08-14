# motorDynamics

Pure-C electric-motor plant models and control foundations for Antshiv
Robotics. The library starts with an armature-controlled brushed DC motor so
the electrical, mechanical, numerical, and ABI contracts can be validated
before adding BLDC/PMSM commutation and inverter models.

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

## Build and validate

```bash
cmake -S . -B build
cmake --build build --parallel
ctest --test-dir build --output-on-failure
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
2. Three-phase inverter and six-step BLDC commutation.
3. Clarke/Park transforms and PMSM field-oriented control.
4. Hall, encoder, and sensorless observers.
5. Coupled motor-propeller and battery-bus fixtures.

