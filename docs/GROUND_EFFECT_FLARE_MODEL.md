# Ground effect and flare-energy model

Landing aerodynamics use a stateful, engine-independent model rather than a
fixed altitude switch or a permanent lift bonus from held brake.

## Ground proximity

The model estimates canopy height from pilot ground clearance and projected
span. A smooth finite-wing proximity curve produces a bounded reduction in
induced drag. This avoids the previous hard eight-metre cutoff and recognizes
that a paraglider canopy remains several metres above its suspended pilot.

Ground proximity does not create energy. It only reduces a portion of induced
drag and increases how effectively a correctly timed flare can convert stored
airspeed into a short landing cushion. Low canopy pressure and collapses reduce
both effects.

## Flare reserve

`FlightState` carries two replay-safe states:

- flare energy, replenished during clean hands-up flight from airspeed and
  canopy pressure; and
- transient flare lift, created by positive symmetric brake travel and then
  decayed.

A decisive brake movement spends finite reserve. Holding deep brake continues
to drain it and accelerates the transient's decay. An early flare therefore
cannot be held indefinitely and repeated pumping cannot manufacture energy.
A late flare with adequate airspeed has more authority than an early held
brake, while slow, unloaded or collapsed canopies have little authority.

The resulting lift increment passes through the same sixteen-panel canopy,
load softening and force integration as normal flight. Brake polar drag and
stall/deep-stall behavior remain active.

## Sensory and analysis output

Telemetry exposes ground proximity, flare reserve, flare authority and the
temporary lift-coefficient increment. Controller feedback adds a restrained
symmetric load pulse, and camera feedback adds a small pitch/compression cue.
CSV telemetry records all four values for landing calibration.

## Validation and limits

Headless tests cover:

- smooth near-ground versus high-altitude behavior;
- finite induced-drag reduction;
- reserve expenditure;
- early held-brake depletion;
- loss of authority with collapse and low pressure; and
- integration through the complete dynamics solver.

The coefficients are research calibration values. Manufacturer data, measured
flare trajectories and blind pilot evaluation are still required before
claiming type-specific landing fidelity.
