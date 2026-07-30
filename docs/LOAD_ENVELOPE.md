# Canopy load envelope

## Structural boundary versus flight behavior

EN 926-1 is a structural-strength qualification standard. Public test reports
describe a sustained test above eight times maximum total weight for a
cumulative three seconds, or an alternative series of higher peaks. That
threshold demonstrates that the specimen survives a prescribed ground-tow
test; it is not a target for ordinary flight or a claim that a low-B canopy
behaves as a rigid wing up to 8 g.

References:

- DIN EN 926-1:2016-02 / EN 926-1:2015, structural-strength scope and test
  sections: <https://www.dinmedia.de/de/norm/din-en-926-1/235033511>
- Air Turquoise EN 926-1 test-report explanation, including sustained and
  peak alternatives:
  <https://www.paraavis.com/site/assets/files/4068/pg_1908-2022_test_toy_m.pdf>
- DHV LTF 2009 English requirements, section 3.2.2 load test:
  <https://www.dhv.de/media/seiten/03_musterpruefstelle/Downloads/Lufttuechtigkeitsforderungen/LTF2009_Eng_final_1.pdf>

## Simulator treatment

The strip solver first computes raw quasi-steady lift. Above a configurable
wing-class onset, a smooth asymptotic response represents billow, arc change,
line compliance and other unresolved flexible-canopy deformation. This keeps
the force continuous while preventing the raw rigid-strip calculation from
remaining near the structural-test boundary for long periods.

The model also applies a wing-class quadratic drag rise above an operational
airspeed onset. This represents the combined growth of porous-fabric, line,
harness, arc-deformation and separated-flow drag that is absent from a simple
low-speed polar. Without it, repeated turns can settle at rigid-wing-like
speeds above 30 m/s. The ten-minute matrix now stays below 27 m/s even in its
deliberately severe scripted cases; ordinary trim/accelerated polars remain
covered separately at their lower target speeds.

Each research wing profile owns:

- `loadSofteningOnsetG`, where progressive deformation begins; and
- `operationalLiftLimitG`, the asymptotic lift-only envelope;
- `overspeedDragOnsetMps`; and
- `overspeedDragQuadratic`.

The 8 g value remains a final structural numerical guard. It is deliberately
separate from the operational model. Total aerodynamic load includes drag and
harness drag, so it can be slightly higher than the lift-only envelope.

The simulator exposes normalized high-load deformation in telemetry and the
expanded HUD. A safety alert appears during strong deformation. The same state
drives a deterministic render contract: subtle span contraction, deeper arc,
line stretch, camber reduction and load ripple. Symmetric fabric/line audio,
controller strain and accessibility-scaled camera compression make the load
readable without changing the physics result.

CSV telemetry exports the deformation together with spanwise shear and
left/right airflow disturbance. Regression runs measure both cumulative and
longest-continuous time above 5 g and reject any case that remains there.

## Validation status

The distinction between structural qualification and operational loading is
standards-backed. The current class-specific softening onset and limit values
are research assumptions, not manufacturer measurements. They require tuning
against instrumented maneuver traces and external pilot/manufacturer review
before any wing profile can be called validated.
