# NXP motor-control portability boundary

The Antshiv six-step controller is split into a portable numerical and safety
contract plus platform adapters. The portable layer does not include NXP
register definitions or copied NXP application source.

```text
portable six-step controller
    alignment -> forced startup -> observed run -> latched fault
                         |
                         v
platform adapter
    PWM + ADC trigger + timer + SPI + hardware fault inputs
                         |
          +--------------+--------------+
          |              |              |
       KV31F           KV58          i.MX RT1176
```

## Reference responsibilities

| Reference | What it contributes |
|---|---|
| AN5169 | Sensorless six-step sequence, startup and commutation evidence for an RC-style BLDC load |
| KV5x quadcopter | Four-motor topology, BEMF/current/bus sensing and four GD3000 stages |
| RT1176 industrial drive | Modern PWM/ADC/SPI integration, four-channel scheduling, GD3000 diagnostics, watchdogs and dual-core partitioning |
| Antshiv ABI | Arming, bounded commands, latched faults, deterministic coast output and flight-controller ownership |

The RT1176 reference uses PMSM field-oriented control. Its peripheral and
GD3000 integration are useful, but they do not replace the sensorless six-step
observer needed for ordinary RC outrunners.

The adapter implements `include/motor/hal/motor_hal.h`. This ABI is deliberately
pre-driver-neutral: GD3000 status bits are translated into generic gate-driver,
current, voltage, temperature, communication, watchdog, and ADC-timing faults.
A future pre-driver can therefore replace GD3000 without changing the portable
controller or the flight-controller command boundary.

The KV31F adapter should use the official MCUXpresso SDK for CMSIS startup,
linker scripts, clocks, pins, FTM/PWM, ADC/PDB, DSPI, and interrupt support.
NXP application-note code remains reference material rather than portable-core
source. The adapter must initialize every bridge output inactive and prove
GD3000 communication and fault readback before accepting an enabled command.

## Current acceptance boundary

The first portable implementation owns the six commutation patterns,
alignment, forced startup ramp, externally validated commutation events,
timeouts, voltage/current bounds and fault latching. BEMF sampling, blanking,
zero-cross qualification and advance timing remain a separate observer. This
keeps an unvalidated observer from silently commanding the power stage.

The real-time controller ABI uses single-precision values to match the
Cortex-M4 floating-point unit. The separate motor plant retains double
precision for desktop simulation and independent-oracle comparisons.
