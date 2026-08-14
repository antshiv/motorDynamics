# BLDC contract

The first BLDC boundary is an averaged three-phase machine model. It retains
phase current, mechanical speed, and mechanical angle as state. Rotor angle
produces three trapezoidal back-EMF waveforms separated by 120 electrical
degrees. Their interaction with phase current produces electromagnetic torque.

The six-step helper emits terminal-voltage vectors for the conventional
`A+B-`, `A+C-`, `B+C-`, `B+A-`, `C+A-`, `C+B-` sequence. It does not yet claim
to simulate MOSFET switching, diode conduction, dead time, PWM ripple, current
chopping, magnetic saturation, or sensorless zero-crossing behavior.

Those effects should become separately testable inverter, commutation, and
observer components. They should not be inserted as unexplained constants in
the accepted machine plant.
