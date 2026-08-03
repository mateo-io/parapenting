# Flight HUD information model

The simulator exposes three persistent HUD modes, cycled with `Tab`.

The cold-boot **Flight Deck** (shown for five seconds at boot, reopened with
`?`, dismissed with `Esc`) is intentionally separate from those
in-flight modes. It is a presentation layer over the route, weather, wing,
graphics, accessibility and input-binding models already owned by the
simulation. It exposes the currently selected value and the live shortcut for
each setting; it does not retain a second copy of any of them.

## Compact (default)

Compact mode keeps only information needed continuously in flight:

- airspeed, vertical speed and height above ground;
- independent brake input and measured handle force;
- active route, target distance, weather preset and simulator wind envelope;
- current training score and coaching feedback.

The left and right brake bars are spatially separated to make asymmetric
inputs readable at a glance. The terrain, canopy and landing references remain
mostly unobstructed.

## Expanded

Expanded mode retains the complete flight-lab diagnostics, setup controls,
riser distribution, replay state, provenance and site-safety text. It is
intended for tuning, training setup and telemetry work rather than normal
flying.

## Minimal

Minimal mode provides one bottom strip containing speed, vario, AGL, brake
steps and landing identity. It is intended for scenic flight and screenshots.

Incident warnings override every mode. Collapse, cravat, spin and deep-stall
cues are never hidden. Landing results and thermal-core cues also remain
available. HUD selection is stored with local pilot preferences and has no
effect on the deterministic flight solver.
