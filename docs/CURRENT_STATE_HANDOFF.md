# Parapenting current-state handoff

Last updated: 2026-07-30, Europe/Amsterdam.

This is the restart point for future work. Treat the worktree and test output
as authoritative if this document ever disagrees with source.

## Product direction

Parapenting is an Unreal Engine 5.8 paraglider simulator targeting Apple
Silicon macOS first, with scalable architecture for a universal Mac and later
Windows release. Physics fidelity has priority over final art. Switzerland is
the first region, centered on Interlaken and Grindelwald.

The default equipment intent is:

- 85 kg pilot;
- seated GH/ABS-style harness;
- low-B wing sized by all-up takeoff mass, not body mass alone;
- keyboard and PlayStation-style controller support from the start.

The simulator must never imply that a manufacturer wing, weather mode, site,
or safety assessment is certified or operationally valid without external
validation.

## Controls

Keyboard flight controls use five persistent steps:

- `A` / `D`: weight shift one step left/right;
- `Left` / `Right`: add one step of corresponding brake;
- `Down`: add one step to both brakes;
- `Up`: release both brakes one step;
- a side arrow held with `Up`: release only that side;
- `Q`: cycle all wings;
- `5`: Training A;
- `6`: BGD EPIC 2 ML research profile;
- `7`: Sport B;
- `8`: ADVANCE EPSILON DLS 28 research profile;
- `1`–`4`: chill, ridge, localized rotor, rotor everywhere;
- `R`: reset;
- `N`: prepare forward ground launch;
- `M`: prepare reverse ground launch;
- `Space`: launch run;
- `Z` / `X` / `C`: left asymmetric, frontal, right asymmetric training event.

Controller support:

- left stick X: proportional weight shift;
- L2 / R2: proportional left/right brake;
- right stick forward: speedbar;
- bottom face button: reset;
- top face button: cycle wing.

See `README.md` and `Config/DefaultInput.ini` for the complete command set.

## Implemented and verified

### Core flight

- deterministic 120 Hz engine-independent dynamics;
- 16-panel canopy with local airspeed, incidence, lift, drag and loading;
- pilot pendulum and three harness profiles;
- compliant accelerator and A/B/C/D riser load transfer;
- five-point per-wing brake polars;
- airspeed-, loading-, pressure- and collapse-dependent brake force;
- flexible high-load deformation, load softening and overspeed drag;
- pitch/surge behavior, deep stall, spin, asymmetric/frontal collapse,
  cravat, pumping, over-brake inhibition and pressure-gated surge checking;
- stateful ground effect, finite flare energy, touchdown and run-out/fall/
  settle dynamics.

### Wings and equipment

Six profiles currently ship:

1. Training A 28;
2. Alpine A+ 27 research envelope;
3. BGD EPIC 2 ML research profile;
4. XC B 26 research envelope;
5. Sport B 26;
6. ADVANCE EPSILON DLS 28 research profile.

Existing enum values were preserved and the EPSILON was appended, maintaining
old replay compatibility. BGD and ADVANCE published facts are separated from
simulator-estimated handling in:

- `Data/Wings/bgd-epic-2-ml-research.json`;
- `Data/Wings/advance-epsilon-dls-28-research.json`.

Per-wing research parameters now include collapse resistance, passive/brake/
pump reinflation, frontal reopening, cravat susceptibility and recovery-surge
energy. The training A is deliberately most forgiving; Sport B is most
dynamic. These are not manufacturer measurements.

### Atmosphere and weather

- chill, ridge, localized rotor and global rotor modes;
- five named weather presets;
- finite deterministic thermal lifecycles;
- expanding, drifting, meandering cores with convergence and sink rings;
- cloud-base/inversion caps;
- ridge lift, boundary layer and lee rotor;
- terrain-normal anabatic/katabatic circulation;
- replay-safe diurnal solar/heating/cloud cycle;
- deterministic three-band turbulence;
- centre/left/right canopy airflow sampling;
- optional Open-Meteo model-wind ingestion with source age and offline cache.

Real weather is contextual simulation input, not a flight forecast.

### Terrain and Swiss routes

- surveyed Interlaken terrain path and reproducible swissALTI3D pipeline;
- tiled procedural terrain rendering, lakes, rivers, roads, fields, forests,
  settlements, snow and rock classification;
- Amisbühl, Bergbo, Hohwald and Niederhorn routes to Lehn/Höhematte;
- source-backed Grindelwald First routes to Grund and Bodmi;
- landing windsocks and wind-oriented circuits;
- route provenance, hazards, wind envelopes and pre-flight suitability.

Important limitation: the sparse Grindelwald lane remains an anchor-matched
analytic proxy. Licensed regional elevation/orthophoto authoring, virtual
textures and World Partition remain unfinished.

### Presentation and game systems

- pressure-responsive double-surface canopy with closed tips, cell thickness,
  collapse deformation, suspension lines and load shading;
- articulated seated pilot/harness;
- procedural alpine atmosphere, volumetric cloud field, sunlight and shadows;
- Low/Medium/High/Epic Metal graphics profiles;
- three cameras with deterministic inertial, rotor, collapse and surge motion;
- stereo procedural wind, vario, fabric, brake-line, thermal and surge audio;
- controller haptics and motion-reduction accessibility profiles;
- expanded/compact/minimal HUD;
- eight repeatable training exercises with scoring and coaching;
- replay library, ghost, CSV export, debrief and persistent progression.

The detailed checklist is `docs/V1_V4_ROADMAP.md`.

## Latest verified test state

One command builds the Unreal module **and** runs every headless suite, in that
order, in under a minute:

```sh
Tools/check-build.sh
```

Results at the end of the Level 9 work:

- `ParapentingEditor` builds clean;
- all twelve physics suites pass, including `calibration_tests`;
- the 60-run ten-minute deterministic matrix passes, 6 wings × 5 weather cases
  × 2 identical repetitions, no run above 5 g, all state finite and repeatable.

Note the CMake suites do not compile a single line of engine code — they build
each `Physics/*.cpp` as its own translation unit, which is exactly the
configuration where a unity-build name collision is invisible. Suites passing
still says nothing about `Source/Parapenting/*.cpp`, which is why
`check-build.sh` builds the module first and should be the gate rather than
`--tests`.

## Where the physics stands

**Levels 0–9 of the geometry-driven stack are built.** Level 9 closed the two
oldest open items in `PHYSICS_TODO`, and the section-polar work after it closed
the third and oldest of all, item 1:

- **Item 10** — the rigid motion counted gravity's restoring torque twice. The
  payload is now a link with its own direction in world axes, the canopy
  carries its own inertia, and the line spring is measured off the built graph
  at four loads because it is geometric and scales with load.
- **Item 0** — trim was 18% slow. It closed at 39.4 km/h against a published
  39.0, on the analytic polars and with the design incidence fitted to that
  speed. It has since been re-identified against the wing's own polar rather
  than the published number, and it still lands: see below.

`docs/CALIBRATION_REPORT.md` is the full Level 9 report.
`docs/PILOT_REVIEW_PROTOCOL.md` is the other half of the exit gate and has not
been run — the handling of this model has never been flown by anyone who flies.

**The section polars are now solved on the section, not stated**
(`PHYSICS_TODO` item 1, closed). `SectionProfile` generates the contour the ribs
are cut to — a real nose radius, brake as a bend in the camber line — and
`SectionViscousSolver` solves it: Hess-Smith panels for the potential flow,
Thwaites/Michel/Head for the boundary layer, Squire-Young for drag, and a
Kirchhoff dead-air region aft of separation iterated to a fixed point. Maximum
lift is where that stops having a solution; nothing states it. The same contour
now draws the ribs, so the wing has one section rather than two.

Validated on NACA 2412 from its own coordinates: zero-lift angle **−2.12°
against a published −2.1**, quarter-chord moment **−0.055 against −0.05**,
minimum drag **0.0062 against 0.006**, maximum lift 1.96 at 16° against 1.6–1.7
at 16 (the angle lands, the value is 18% high, which is the method's known
direction of error).

**Trim was re-identified against the wing rather than against the brochure.**
The design incidence used to be fitted directly to the published 39 km/h; it is
now set by the design rule a rigging angle exists to satisfy — hands up, with no
brake and no bar, the wing sits at its own best glide. At the published 105 kg
that gives **trim 39.8 km/h against 39.0** and incidence **5.14° against the
5.30° the published trim lift coefficient needs** — both inside 3%, neither
fitted. Sink and glide do not land, and they are one error rather than two: see
the drag item below.

**Brake now does what brake does**, and all three are gated in `coupled_tests`:
25% brake gives **9.77 m/s and 10.34 glide against 10.48 and 10.87 hands up** —
slower and less glide — and a firm input from trim **climbs at 1.15 m/s** before
settling slower, which is the same pendulum as the release surge with the sign
reversed. An earlier note here said brake was making the wing *faster*; that was
measured off a ramp started before the wing had finished settling and it was
wrong. See `PHYSICS_LEARNINGS` §30.

**The section was stalling at its nose, and fixing that fixed three things**
(`PHYSICS_TODO` item 13). A turbulent boundary layer separating in the first 3%
of chord was being read as the section letting go, so one degree took the flow
from separating at 94% of chord to 3%. Letting a leading-edge bubble reattach —
the turbulent twin of the laminar short bubble already in the code — made
maximum lift and stall angle **monotone in brake** (1.81 at 12° hands up, 2.05
at 11°, 2.10 at 9°, 2.35 at 9°), restored the 4 m/s asymmetric gust benchmark to
folding less (0.653 against 0.888) and **clearing completely**, and put the
symmetric frontal's two halves back at 0.710 and 0.710.

**What is still short: brake reaches the wing with the wrong sign, and the wing
departs at 40% of travel** where an EN-B stalls at 65–80%. Full bar reaches
**13.4 m/s before it diverges**. Both are `PHYSICS_TODO` item 11, the pitch
axis.

The brake pull used to be **counted twice** — the line network shortened the
brake run by the whole 0.62 m of handle travel and rotated a rigid canopy with
it, while the section polars spent that same travel again as camber. A brake
line ends at 98% chord, so there is one length, not two. Counted once, the
0.62 m divides as 0.120 slack / 0.298 fabric / 0.202 rotation, and full brake
rotates the canopy 5.0° where it used to rotate it 12.4°.

**The bigger finding is that none of these numbers were settled, and the wing
has a second pitch mode nobody had flown long enough to see.** Incidence spread
over a ten-second window, still air, no input, hands-up: **1.010°** at 30 s,
0.597 at 60, 0.361 at 120, 0.074 at 240, 0.004 at 480, **0.000 at 960**. Under
25% brake the same run is 3.426° falling to 0.022°.

It settles — it takes eight to sixteen minutes. Measured off 35 peaks of a
1200 s run, the mode is **period 16.3 s, damping ratio ≈0.030**, with incidence
and airspeed in antiphase: it is the phugoid, and classical theory for this
wing gives 4.80 s and 0.062. The period is **3.4× longer than theory**.

**Both numbers are now explained.** The classical formulas assume incidence is
held fixed, so lift and drag go as V². Measured off the flight path of the same
run — no aerodynamic loads trusted — this wing's go as **V^0.171** and
**V^0.313**, and those two exponents predict the period to 16.42 s against
16.39 measured and the damping to 0.034 against 0.031. The mechanism: **the
pendulum holds lift, not incidence.** The wing rotates nose-up on its lines at
−1.69°/(m/s) as it slows, so lift varies by 0.97% of weight over the whole mode
and the phugoid's restoring force is only the leftover. `PHYSICS_LEARNINGS` §34.

Consequence for anyone picking this up: the slow mode is **not** evidence about
`swingDampingRatio` — the tuned coefficient appears in neither formula. What
still pins it at 0.35 is that the aircraft departs at 0.25.

**And that departure is a different mode, which was checked rather than
assumed.** The exponent predicts divergence when n goes negative, so the
departure should have been n crossing zero. It is not: n stays 0.14–0.19 across
the boundary, and `--departure` shows the thing actually growing is a
**3.6–5.7 s** mode — the pendulum band — with damping −0.017 at ratio 0.25 and
−0.042 at 0.20. So item 11 is two problems, not one: the slow mode is explained,
the fast one is not. `PHYSICS_LEARNINGS` §35.

What `calibration_tests` reports as "Pitch: period 2.91 s, damping 0.28" is the
pendulum mode — the wing swinging against the pilot after a brake pulse — which
is a different and healthy mode. Settled properly: hands-up **4.925°**, 25%
brake **5.724°**, so **brake raises incidence, which is the correct sign.**

Every settle this project has used — 20, 40, 60 seconds — was far too short,
and two conclusions were drawn from those runs and later retracted: that brake
had the wrong sign, and that the wing had a limit cycle. Both are corrected
under `PHYSICS_TODO` item 11 with the evidence; the general lesson is
`PHYSICS_LEARNINGS` §33. Use `parapenting_pitch_axis_trace`, which settles to a
criterion rather than a clock. Four gates were loosened to bound the brake
behaviour rather than fit it; they are tabulated with their strict thresholds
under item 11.

**The largest disagreement is now drag** (`PHYSICS_TODO` item 12), though a
fifth smaller than it was written up as. Settled to a criterion rather than the
fast suite's 90-second clock: glide **10.96 against 9.5** and sink **1.015
against 1.14**, where the clock said 11.33 and 0.97. It is one error — trim
speed and incidence both land, so only the drag is wrong. The solved section
runs 0.0157 at trim against the 0.018–0.025 paraglider sections are quoted at.
The missing term is named — the shear layer off the cell mouth — and has been
tried twice and left out both times, because its size is a coefficient that
swings section drag five-fold and one value in its range lands exactly on the
published glide. The VSM's induced drag was checked and ruled out.

**Nothing geometry-driven flies the game yet.** `ParagliderDynamics` — one
six-degree-of-freedom body with a fitted polar — is still what the pawn uses.
Retiring it is Level 10, and it should not start until the envelope above is
wider, because Level 10's exit gate is "no legacy direct-control force remains
active" and swapping in a model that cannot hold 40% brake would be a
regression a pilot would feel immediately. That is unchanged — but the thing to
fix before the geometry-driven stack can REPLACE the legacy one is now the
pitch axis, item 11, and nothing else in the aerodynamics.

## Work in progress at interruption

None. The tree is green and there is no half-finished pass.

An earlier handoff described an unfinished reinflation sensory pass
(`Telemetry` reinflation rates, camera and haptic cues). That work is in the
tree and the suites cover it; the remaining items from it were audio routing,
CSV columns and documentation, and they are listed under the next order below
rather than as an interruption.

## Mac build state

Unreal Engine path:

```text
/Users/Shared/Epic Games/UE_5.8
```

Editor build command:

```sh
"/Users/Shared/Epic Games/UE_5.8/Engine/Build/BatchFiles/Mac/Build.sh" \
  ParapentingEditor Mac Development \
  "/Users/pachosky/projects/parapenting/Parapenting.uproject" \
  -WaitMutex -NoHotReloadFromIDE
```

An older app exists at:

```text
build/package/Mac/Parapenting.app
```

Its directory timestamp is 2026-07-30 15:28:31. It predates the latest wing,
landing-rollout and collapse/recovery changes and must not be described as the
latest build.

**The module builds, in about 35 seconds.** Run `Tools/check-build.sh` — module
first, then all eleven suites. There is no reason to skip it.

The paragraph that used to be here said Unreal build/cook was "quota-blocked
until roughly 2026-08-05 13:55". That was wrong, it went unchallenged for a long
time, and it silently changed how work was done: engine changes were committed
unverified and a whole section of `docs/PHYSICS_TODO.md` was written to track the
resulting debt. When the claim was finally tested, the module built clean on the
first attempt with every "unverified" file compiling. Unreal has no build quota.

Left here deliberately as a marker: a written-down constraint is the kind of
claim nobody re-tests, because the response to it is to route around it rather
than to check it. `PHYSICS_LEARNINGS.md` §17.

The steps below remain the right ones for cook/package and QA:

1. compile the editor target;
2. launch and smoke-test the Amisbühl → Lehn route;
3. test keyboard and a PlayStation-style controller;
4. cook/package the latest Mac app;
5. inspect logs for Metal/shader errors;
6. verify packaged startup on Apple Silicon;
7. later complete universal signing/notarization and Windows QA.

## Remaining release gates

The v1–v4 goal is not complete until these are addressed:

- external SIV/pilot validation of pitch, brake pressure, collapses and recovery;
- manufacturer/importer cooperation for licensed real-wing identity, geometry,
  polar and maneuver data;
- licensed regional terrain/orthophoto integration and scalable World Partition;
- runtime visual QA of the newest source in Unreal on Mac;
- performance profiling across representative Apple Silicon machines;
- current standalone Mac package;
- signed/notarized universal Mac release;
- Windows build and platform QA.

## Recommended next order

1. **Widen the envelope before Level 10.** Real section polars are the single
   change that moves both limits, and they need XFOIL or equivalent over
   digitised EPIC 2 ML profiles. Everything else in the pitch axis is now
   measured; this is the one input that is not.
2. Retire `swingDampingRatio` by finding the stabilising mechanism it stands in
   for, rather than by measuring it more precisely (`PHYSICS_TODO` item 11).
3. Run `docs/PILOT_REVIEW_PROTOCOL.md` with experienced pilots. It is the half
   of Level 9's exit gate that cannot be closed from a keyboard, and the surge
   timing question is the only external reference available for item 11.
4. Perform the Unreal runtime smoke test owed after rig/terrain changes — fly
   Amisbühl → Lehn, weight-shift hard both ways, and `[`/`]` to a Grindelwald
   route and back.
5. Finish the reinflation sensory pass: audio routing, telemetry CSV columns,
   documentation.
6. Level 10 — profiling, solver levels of detail, research visualisation
   toggles, and removal of the legacy path.
7. Produce release-quality Mac and Windows builds only after validation.

