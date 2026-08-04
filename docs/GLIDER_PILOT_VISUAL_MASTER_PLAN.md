# Glider and pilot visual master plan

Last reviewed against the source: 2026-08-04

## North star

The pilot, harness, controls, risers, suspension lines and canopy must read as
one continuous load-bearing machine.

At any useful chase-camera distance the player should be able to see what the
pilot is doing, trace that action through the brake handles and lines, and see
the corresponding response in the trailing edge and wing. Nothing may appear
to float, stretch arbitrarily, terminate in mid-air or react before the part
that caused it.

This is a presentation plan. Flight physics remains authoritative and fixed
step. Rendering interpolates immutable snapshots and adds only bounded,
cosmetic high-frequency motion.

## Why it looks fake today

The project already owns much better physical data than it presents. The
runtime has an authoritative canopy shape, an explicit suspension graph,
left/right brake travel and force, riser-group load, harness pitch and roll,
weight shift, pressure, collapse and recovery state. The visible chain loses
credibility because:

- the pilot is assembled from Engine cubes, cylinders and spheres rather than
  a human skeleton with stable joint lengths and anatomical limits;
- `PilotPose` returns only a rig offset/rotation plus shoulder, elbow and hand
  points per side — no pelvis, chest, head, knee or ankle targets, no wrist
  orientation, grip state, hand inertia or two-handed coordination;
- brake handles are scaled spheres, not loops held by fingers;
- risers exist as drawn geometry — `RiserTopLocalCm` places four groups
  (A, A', B, C) 45 cm above the carabiner with a fore/aft spread, drawn as two
  webbing rails plus a maillon bar, with harness and shoulder webbing below —
  but the webbing is a straight rail pair that cannot bend or twist, and there
  is no brake pulley, keeper or accelerator routing;
- the suspension renderer (`AddSuspensionSegment`) turns every segment into a
  four-sided tube with a fixed world-space radius, which makes sub-millimetre
  line groups read like rods at normal camera distance and gives no
  screen-space thickness floor at range;
- main lines route through the solver's cascade junction, but the brake fan is
  a parallel decorative plan: four hardcoded branches per side at span
  `0.24 + 0.20 * branch`, a lerped mid-point, and a trailing edge built from
  local constants rather than from the authoritative attachment list;
- presentation reads commanded input (`AppliedControls.leftBrake` /
  `rightBrake`) in several places where it should read achieved brake travel,
  so the wing and lines can move before the control that caused them;
- canopy, cascade and hand motion are updated as presentation subsystems, not
  sampled from one time-coherent rig snapshot;
- cloth deformation communicates global state, but local line pull is not yet
  visibly carried into the matching trailing-edge stations;
- there is no dedicated camera composition that keeps hands, risers, lines and
  enough of the wing readable together.

The first target is therefore not "a better character" or "more cloth" in
isolation. It is continuity of cause, geometry and timing through the complete
chain.

## The one rig contract

Every rendered part consumes one interpolated `GliderRigSnapshot` produced at
the fixed simulation boundary:

```text
pilot controls
    -> body / harness pose
    -> left and right hand grips
    -> brake handles and riser anchors
    -> main, cascade and brake-line paths
    -> canopy attachment points
    -> local trailing-edge and cell deformation
```

The snapshot should contain:

- canopy and payload transforms plus their relative pitch and roll;
- pelvis, chest, head, shoulder, elbow, wrist, knee and ankle targets;
- left/right grip position, rotation, travel, velocity and brake force;
- both carabiner transforms and all physical riser-top transforms;
- every visible suspension node position, tension, slack and row identity;
- every canopy attachment transform and local skin displacement;
- pressure, separated span, collapse, load factor and recovery surge;
- simulation timestamps for previous and current snapshots.

The renderer interpolates positions and rotations at render time. Discrete
events such as taking a wrap, releasing a handle or clipping in use explicit
state transitions rather than being inferred from one frame of coordinates.

## Scale and readability rules

Physical dimensions are the source of truth. Visual exaggeration is allowed
only through a named distance LOD:

- Hero: correct line diameters, individual risers, hardware and hand grips.
- Chase: thin screen-space lines, simplified hardware, complete causal motion.
- Far: grouped line ribbons and pilot silhouette; never floating endpoints.
- Replay/photo: hero geometry, seam detail and deterministic secondary motion.

Line visibility must be solved with anti-aliased screen-space thickness,
lighting and contrast, not by making every line a centimetre-scale pipe.

## Delivery ladder

Each stage leaves a shippable improvement and includes a comparison capture
from the same deterministic replay.

### Stage 0 — truth capture and composition

- Record neutral, full-left, full-right, symmetric flare, weight-shift,
  collapse and recovery replays.
- Add a temporary rig diagnostic view naming hands, carabiners, riser tops,
  cascade nodes and canopy attachments.
- Define three evaluation cameras: rear chase, three-quarter close chase and
  side technical view.
- Measure current endpoint gaps, limb-length changes, line penetration and
  input-to-visual latency.

Exit gate: golden captures and measurements exist for every reference replay;
the close chase view contains pilot, both hands, risers, line fan and canopy.

### Stage 1 — continuous kinematic chain

- Introduce `GliderRigSnapshot` and render interpolation.
- Make hands, brake handles, brake-line roots, risers and canopy attachments
  consume the same timestamped snapshot.
- Replace input-driven hand travel with achieved brake travel; expose command
  lag through the arm rather than allowing the hand and line to disagree.
- Enforce constant upper-arm, forearm and riser lengths.
- Add joint limits and pole-vector targets so elbows cannot flip.

Exit gate: zero visible endpoint gaps in all reference replays; no limb or
riser changes length; a brake handle never moves before its hand.

### Stage 2 — production pilot and harness

Asset sourcing, licensing, acceptance criteria and the Mannequin blockout path
that unblocks this stage before the asset exists are in
[PILOT_CHARACTER_ASSET_GUIDE.md](PILOT_CHARACTER_ASSET_GUIDE.md).

- Replace primitives with a licensed, retargetable skeletal pilot at plausible
  seated scale, including helmet, clothing, footwear and hands.
- Build a harness mesh with seat, back protection, shoulder straps, leg straps,
  reserve volume and carabiner hang points.
- Drive pelvis from payload motion, chest and head from filtered inertial
  response, and limbs through full-body IK.
- Add seated, launch-run, landing-run, flare, impact and fallen pose families
  with inertial blending between them.
- Keep face work minimal; silhouette, posture, grip and load response matter
  first.

Exit gate: no Engine primitive is visible in the live pilot; pose transitions
have no pops; hands remain attached under full harness roll and pitch.

### Stage 3 — hands, controls and riser webbing

- Model left/right brake loops with a real grip and wrist orientation.
- Carabiners, maillons and A / split Baby-A / B / C risers already exist as
  drawn geometry; add the brake pulley, keeper magnets/snaps and accelerator
  lines, and give the hardware real cross-sections.
- Skin riser webbing so it bends and twists while maintaining length, replacing
  the current straight rail pair.
- Animate fingers between open, acquire, wrapped, loaded and released states.
- Make brake force alter wrist, elbow and shoulder effort without changing
  achieved travel.

Exit gate: every line can be visually traced to real hardware; the left and
right systems remain independent; hands-up, half-brake and flare read without
HUD assistance.

### Stage 4 — suspension line renderer

- Render the authoritative graph, including mains, cascades, upper galleries
  and brake fan; never author a parallel decorative line plan. Concretely:
  retire the hardcoded four-branch brake fan and its local trailing-edge
  constants, and take brake attachments from `SuspensionGeometry` the way the
  main lines already do.
- Replace the fixed world-space tube radius with a screen-space width floor so
  a line stays visible at range without becoming a rod up close.
- Use camera-facing analytic lines or ribbons with stable sub-pixel coverage,
  row-aware colour and physically based highlight response.
- Derive sag from solved slack and tension. Add only small deterministic
  flutter whose amplitude is gated by slack and apparent airflow.
- Prevent intersections at cascade junctions and eliminate temporal shimmer.
- Provide hero, chase and far LODs with cross-faded group simplification.

Exit gate: lines survive a Shipping build, remain readable at chase distance,
do not shimmer in motion and meet every hand, riser, cascade and canopy node.

### Stage 5 — canopy as sewn, pressurised fabric

- Upgrade to cell/rib topology with distinct upper and lower skins, open
  leading-edge intakes, closed tips and a continuous trailing edge.
- Add panel seams, diagonal/rib structure, reinforcements and subtle ripstop
  normal response at physically plausible scale.
- Skin all authoritative attachment points into the fabric topology.
- Map brake fan shortening into local trailing-edge displacement at the actual
  attachment stations before smoothing displacement through neighbouring
  cloth vertices. This replaces the current uniform `Brake * 55 cm` drop
  applied identically to every branch of a side.
- Map pressure, span loading and collapse to a small set of stable deformation
  modes; avoid arbitrary noise and rubber-sheet stretching.

Exit gate: asymmetric brake visibly starts at the matching hand, travels
through its line fan and pulls only the matching trailing edge; cell volume is
preserved in normal flight and folds remain attached to line geometry.

### Stage 6 — human secondary motion

- Add filtered breathing and head-look, suppressed under high workload.
- Let shoulders, loose clothing and harness straps respond to measured load
  direction and acceleration.
- Add hand micro-corrections from achieved control velocity and line force,
  deterministic in replay.
- Couple legs and torso to weight shift so the move originates in the pelvis,
  not as a whole-body lateral translation.
- Add physically limited recovery reactions without scripted SIV choreography.

Exit gate: neutral flight is alive but quiet; turbulence increases corrective
activity; identical replays produce identical hero-camera motion.

### Stage 7 — camera and player readability

- Author a glider-focused chase camera that composes the pilot low-centre and
  canopy high-centre while preserving terrain sight lines.
- Add a close technical view for hands, risers and brake travel, and a side
  view for pitch/surge analysis.
- Use existing `CameraFeedback` for filtered motion and respect Full, Comfort
  and Minimal Motion accessibility modes.
- Adjust framing with canopy–payload separation rather than changing physical
  transforms or hiding the wing.

Exit gate: hands-up, one-sided input, flare, weight shift and surge can each be
identified in a two-second silent clip with the HUD off.

### Stage 8 — incidents and edge cases

- Define line and body presentation for slack, frontal/asymmetric collapse,
  cravat, deep stall, spin, reinflation, launch inflation and failed launch.
- Keep brake lines, risers and limbs stable when load approaches zero.
- Add contact-aware handling for ground, canopy and line proximity without
  letting presentation collision affect flight physics.
- Validate transitions at low frame rate, paused replay and time dilation.

Exit gate: no NaNs, teleporting joints, exploding cloth, infinite flutter or
detached endpoints across the incident replay matrix.

## Tests and review protocol

### Headless contracts

- Snapshot interpolation is bounded and never extrapolates past its limit.
- Limb, webbing and suspension segment lengths stay within tolerance.
- Left input changes no right grip or right brake attachment target.
- Grip travel equals achieved brake-line take-up after free play.
- Every visible line endpoint resolves to an existing rig node.
- No presentation path reads `AppliedControls` directly; control-derived visuals
  come from achieved telemetry only.
- Cosmetic oscillation is a function of simulation time, never wall clock or
  frame count, so a replay at any frame rate produces identical geometry.
- Presentation has no effect on solver state hashes or replay trajectories.

### Automated visual checks

- Golden screenshots from all three cameras and reference replays.
- Endpoint-gap and silhouette masks.
- Motion-vector checks for line shimmer and pose discontinuity.
- Shipping-build captures at Low, Medium, High and Epic scalability.
- 60, 30 and uneven-frame-time playback of the same fixed-step replay.

### Human review

Review in this order:

1. Can a pilot identify control state with the HUD off?
2. Can they trace every visible load path without finding a floating end?
3. Does posture communicate weight shift, brake force and surge correctly?
4. Does the wing deform where the corresponding lines pull it?
5. Only then: are materials, seams, clothing and micro-motion attractive?

Use structured review from experienced paraglider pilots for posture, riser
routing, grip and incident behaviour. Label visual and physical approximations
until reference video or instrumented data supports them.

## Immediate build order

The next implementation slice should be Stage 0 plus Stage 1, not a character
asset purchase. It fixes the causal and timing foundation that every later
asset depends on:

1. Define the timestamped snapshot and interpolation tests.
2. Move achieved hand/grip, carabiner, riser and attachment transforms into it.
3. Add diagnostic nodes and the three evaluation cameras.
4. Record the reference replay matrix and baseline captures.
5. Remove endpoint gaps and limb-length changes before replacing meshes.

After that foundation is stable, Stage 2 and Stage 3 can be produced in
parallel as character/harness art and control-hardware art, then integrated
against the same rig contract.

Stage 2's licensed character is a procurement decision, not a code dependency.
Do not let it block the stage: build the harness mesh, IK rig, retargeter and
pose-family state machine against the UE5 Mannequin, and swap the pilot in as a
data change once it is approved. See
[PILOT_CHARACTER_ASSET_GUIDE.md](PILOT_CHARACTER_ASSET_GUIDE.md).

## Definition of done

This track is complete when a HUD-free replay makes the whole action legible:
the pilot shifts or pulls, the correct arm and hand carry the effort, the real
handle and riser system move, the matching line groups tension or slacken, and
the corresponding part of the sewn canopy responds with no discontinuity in
space or time.
