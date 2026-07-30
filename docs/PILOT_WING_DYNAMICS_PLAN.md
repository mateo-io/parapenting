# Pilot–wing dynamics plan

Updated 2026-07-30.

## Target

The simulator should model a paraglider as a coupled canopy, suspension system,
and harness/pilot payload. Terrain and weather remain inputs; the priority is
what the pilot sees, feels, and can cause through brakes, accelerator, and
weight shift.

The BGD EPIC 2 ML is the reference geometry. Published dimensions are treated
as facts; attachment positions digitized from diagrams and all aerodynamic
coefficients remain research estimates until measured or manufacturer
validated.

## Reference geometry

- 27 m² flat area, 22.8 m² projected area
- 11.8 m flat span, 9.3 m projected span
- 2.8 m root chord, 45 cells, flat aspect ratio 5.2
- 7.3 m canopy-to-riser height and 254 m total line length
- three main-line counts per side: 3 A, 4 B, 3 C
- pilot-side risers: A plus split Baby-A, B, and C
- upper canopy rows: A/B/C/D; D cascades into the rear C system
- independent brake/K cascade to the trailing edge
- trim risers 500 mm; full accelerator A/A′ 380 mm, B 420 mm, C 500 mm
- ML brake range 720 mm at maximum all-up weight

## Incremental implementation

### Five-pass pilot-input iteration — implemented 2026-07-30

1. The left and right A/A′, B, C/D-cascade, and brake groups now carry
   independent tension and slack state.
2. Weight shift moves both carabiners, redistributes suspension load, and
   drives bank through measured load imbalance instead of direct key torque.
3. Rendered risers follow the carabiners. Each group changes thickness and
   color with load and develops catenary-like sag/flutter when slack.
4. Each canopy half reports separated span. Separation progressively removes
   roll authority, so deep brake transitions toward spin/parachutal stall
   instead of continuing to roll.
5. Brake travel is rate-limited independently on each side. A commanded pull,
   progressive application, hold, and release are distinct physical states.

The automated maneuver contract covers left/right symmetry, moderate held
turns, deep asymmetric braking, symmetric deep stall, brake travel/release,
line unloading, weight-shift load transfer, and recovery. These are research
envelopes and still require instrumented-flight and expert-pilot validation.

### Fundamental lateral/longitudinal correction — implemented 2026-07-30

- Weight shift now changes semispan section loading as a first-order
  approximation of line-driven arc deformation, in addition to moving the
  payload and redistributing carabiner load.
- Bank produces a coordinated yaw-rate target
  `g * tan(bank) / airspeed`, with the simulator sign convention applied.
  This curves the flight path under weight shift alone.
- The yaw/bank feedback is gated by active pilot input so releasing the
  controls restores the wing instead of trapping it in an artificial spiral.
- Symmetric/common brake deflection adds lift to both semispans without
  reintroducing asymmetric brake-roll torque.
- Fast brake application at elevated airspeed charges a transient,
  energy-limited zoom state. It increases lift while the wing pitches back,
  producing positive vertical speed and an altitude peak before held-brake
  drag, separation, and stall dominate.
- Deep separated span reduces the allowable roll rate and bank envelope.

Tests now require full weight shift alone to generate bank, heading change,
and lateral displacement. A separate high-speed brake test requires an
airspeed-to-height exchange followed by descent under held brake.

### Independent lateral suspension mode — implemented 2026-07-30

- Canopy roll relative to the payload is now an independent state with angle
  and angular rate.
- Its natural frequency is derived from the 7.3 m suspension height.
- Carabiner load imbalance moves the relative-roll equilibrium; raw
  weight-shift input no longer sets system bank directly.
- Relative canopy roll is transmitted into bank only through supported line
  tension and attached span.
- Low pressure, frontal collapse, slack groups, and separated span reduce
  lateral control transmission.
- The procedural canopy and every load-bearing/brake endpoint rotate around
  the payload using the relative-roll state.

Tests compare identical full weight shift on loaded and unloaded systems and
require the unloaded canopy to transmit less control and develop less relative
roll.

### 1. Suspension geometry and visible rigging — implemented

- explicit EPIC 2 attachment groups and chord/span locations
- A/A′, B, C, and brake cascades rendered separately
- published line length contributes aerodynamic drag
- accelerator and brake move load between the actual riser groups
- resultant attachment load contributes a pitch moment
- D-row compatibility telemetry is retained as zero, because D is not a
  fourth pilot-side riser on this wing

### 2. Coupled pilot/canopy bodies — next

Replace the current single rigid-body plus harness spring with an 8-DOF model:
six canopy/system degrees of freedom plus independent payload roll and pitch.
Track canopy and payload mass/inertia separately. Resolve line tension at the
risers, enforce suspension length, and apply equal/opposite forces to canopy
and payload. Weight shift becomes lateral payload-CG displacement rather than
a direct roll moment.

Acceptance tests:

- pendulum period scales with suspension length and payload position
- a brake pulse produces pitch-back/climb, then a surge on release
- total energy and momentum remain bounded with no aerodynamic input
- changing pilot mass changes tension and response without changing geometry

### 3. Distributed controls and stall

Map left/right brake travel into spanwise trailing-edge deflection. Use
section-local incidence and airspeed for separation instead of a global brake
threshold. Add hysteresis for stall entry/recovery, line unloading, backward
canopy flight, and asymmetric spin.

Acceptance tests:

- brief deep input can flare without becoming a sustained stall
- held deep input produces parachutal/deep stall and loss of pressure
- release timing changes surge magnitude
- one-sided stalled panels produce autorotation and reduced brake pressure

### 4. Flexible canopy and collapses

Make line tension and cell pressure drive attachment positions and projected
area. Collapse initiation must arise from local negative incidence/unloading;
reopening must follow local pressure, spanwise propagation, brake pulses, and
payload motion. No recovery input is scripted.

### 5. Validation

Build repeatable trim, brake-step, pitch-pulse, roll-step, spiral-exit, stall,
and collapse cases. Fit only to licensed manufacturer data or instrumented
flight tests. Keep every unvalidated coefficient labelled as research.

## Source basis

- BGD, *EPIC 2 Owner's Manual*, version 7, February 2026, especially technical
  data, risers, accelerator/brake ranges, and line plan.
- Cumelles Céspedes et al., “An eight-degree-of-freedom coupled aerodynamic
  model for high performance paraglider-harness/pilot systems,” *Aerospace
  Science and Technology* 169 (2026), 111440.
- P. F. Heatwole, *Parametric Paraglider Modeling*, component and
  demonstration chapters: canopy, suspension-line, accelerator, brake, and
  payload-CG models.

## Ten-pass research architecture iteration — 2026-07-30

The current implementation is now a research-oriented, deterministic
prototype. It is not yet a validated flight-training device; coefficients
still need fitting against instrumented EPIC 2 ML flights and structured
pilot/SIV review.

1. Split the published canopy mass from payload mass.
2. Add anisotropic translational apparent mass in canopy body axes.
3. Add apparent rotational inertia to roll, pitch, and yaw response.
4. Resolve left/right suspension load imbalance into roll and yaw moments.
5. Apply suspension moments as relative canopy/payload reactions instead of
   hiding them in direct control torque.
6. Replace instantaneous stall clipping with per-side separated-span state,
   with deliberately different entry and recovery time scales.
7. Filter A/A′, B, C, and brake tension/slack independently, then make
   collapse recovery depend on actual supporting-line and brake-line load.
8. Add deterministic 120 Hz weight-shift, brake-zoom, symmetric deep-stall,
   and asymmetric-stall protocols with quantitative result metrics.
9. Record aerodynamic force, gravity, acceleration, mechanical energy,
   aerodynamic power, and the integration/model energy residual.
10. Render separate risers, carabiners, cascades, and brake fans. Line sag,
    flutter, colour, and thickness now expose filtered slack and tension
    history during incident review.

The maneuver tests enforce qualitative physical invariants, not claimed
manufacturer performance figures: weight shift must bank and displace the
system laterally; a high-speed brake application must first exchange speed
for height; sustained symmetric deep brake must produce separated span and
low airspeed; asymmetric stall must remain bounded instead of devolving into
an unlimited game-like barrel roll.
