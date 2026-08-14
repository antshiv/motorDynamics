# Brushed DC motor contract

The accepted baseline separates the motor plant from its eventual drive and
controller. Terminal voltage and external shaft-load torque are explicit
inputs. Current, angular speed, and shaft angle are persistent states.

The model assumes constant parameters, linear magnetic behavior, a rigid
shaft, and viscous friction. It intentionally does not yet model switching,
dead time, Coulomb friction, saturation, temperature, gearbox compliance, or
battery voltage sag. Those effects must enter as visible extensions rather
than undocumented correction factors.

The sign convention is positive terminal voltage, current, electromagnetic
torque, speed, and angle in the declared shaft direction. Positive load torque
opposes positive rotation.

