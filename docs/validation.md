# Validation

The baseline is accepted only when all of these boundaries pass:

| Gate | Evidence |
|---|---|
| Continuous equations | Known initial derivative fixture |
| Static equilibrium | Closed-form current and speed solution |
| Time integration | RK4 convergence to the static equilibrium |
| Independent oracle | C trace compared with SciPy `solve_ivp` |
| Failure behavior | Invalid configuration clears outputs and preserves state |
| Runtime safety | AddressSanitizer and UndefinedBehaviorSanitizer CI |

Future machine models should keep their own independent fixtures. A BLDC or
PMSM implementation must not be accepted merely because it produces plausible
rotor speed in the complete aircraft simulation.

