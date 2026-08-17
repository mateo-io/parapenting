# Performance: the frame, not the solver

The symptom is fans, on a fast machine, at a moment when nothing new has been
added and the next thing to add is wind and lift. This is the plan for getting
the headroom back, written the way the physics ladder is written: levels, gates
and tests, one knob at a time, and no number in it that has not been measured
by the level that owns it.

## The one thing already measured, and it reframes everything below

`SOLVER_PROFILE.md` and `SOLVER_LOD.md` measured the flight solver on this
class of machine:

| | µs/step | share of a 120 Hz budget |
|---|---|---|
| full-fidelity schedule | 540 | **6.5% of one core** |
| reduced-fidelity tier | 231 | 2.8% of one core |

**So the fans are not the physics.** A plan that opened by optimising the
solver would be tuning 6.5% of one core while something unmeasured burns the
rest, and this project has a documented history of exactly that mistake — the
profile's own "the cost is the lines, not the air" section exists because
everyone assumed the aerodynamics dominated and they were 36%.

**And nothing has ever measured the frame.** There is no `FRAME_PROFILE.md`.
Every cost outside `CoupledParagliderSolver` — rendering, the game thread, the
HUD, the terrain mesh, the atmosphere sample — is currently unknown, not small.
This document must therefore be read as: one measured section, then a ladder of
suspects in the order they are worth measuring.

## Three structural facts found by reading, before any measurement

These are not profiling results. They are things the configuration says, and
each one names a level below.

1. **There is no frame-rate cap anywhere.** `Config/DefaultEngine.ini` sets
   `bUseFixedFrameRate=False` and no `t.MaxFPS`; no vsync setting is applied in
   `Source/Parapenting`. An uncapped renderer on a fast machine will produce as
   many frames as the GPU can make, indefinitely, at 100% power. This is the
   single most likely explanation of the symptom and the cheapest to test.
2. **The terrain is 64 procedural mesh sections with no LOD.** 8 × 8 tiles of
   50 × 50 cells (`TerrainRenderLayout.h`) is ~2600 vertices and 5000 triangles
   per section, ~320k triangles total, drawn at full density at every distance,
   because `UProceduralMeshComponent` has no LOD chain.
3. **The atmosphere sample is outside every profile taken so far.**
   `AirModel.SampleCanopy` runs once per fixed step from the pawn, and
   `AtmosphereModel.cpp` is 25 KB of gusts, thermals, ridge lift, slope
   circulation and rotor. `Tests/SolverProfile.cpp` constructs a
   `CoupledAtmosphere` and never calls it, so this stage's cost has never
   appeared in any table. **It is also precisely the stage the wind and lift
   work will grow**, which makes it the one place where "we are not at the
   limit" is currently a hope rather than a measurement.

## How this plan is allowed to be wrong

The same rules the physics ladder runs on, because they are what make a plan
retractable instead of load-bearing:

- **Measure before cutting.** Every level below states its instrument first and
  its change second. A level that cannot state what it would measure does not
  start.
- **One knob per row.** `SOLVER_LOD.md`'s sweep is the model: one change off
  the reference, both signatures flown, cost and consequence in the same table.
- **No wall-clock assertion enters `check-build.sh`.** Wall clock is a property
  of the machine; gating on it turns a busy laptop into a red suite. Profiles
  assert nothing and live in their own binaries. What *can* be gated is
  structure — that a cap is set, that terrain heights are unchanged, that a
  frame does no heap allocation it did not do before.
- **Visual changes are gated by `VISUAL_QA.md`, not by opinion.** Anything at
  Level 3 or below changes what the pilot sees, and "it still looks fine" is
  not a gate.

---

## Level 0 — measure the frame (blocks every level below)

Nothing here changes behaviour. It produces the document the rest of the plan
is read against.

**Deliverable:** `docs/FRAME_PROFILE.md`, in the shape of `SOLVER_PROFILE.md` —
named machine, named build configuration, named scene, and a table that adds up.

**Protocol**, on a standalone build rather than in the editor (the editor is
itself a large and irrelevant cost, and measuring it would put the wrong number
in the table):

- `stat unit` — the four numbers everything else is attributed against: Frame,
  Game, Draw, GPU. This alone decides whether the problem is CPU or GPU, and
  every level below is ordered by that answer.
- `stat scenerendering`, `stat gpu` — where the GPU time goes, if it is GPU.
- `stat game`, and `stat dumpframe` — where the game thread goes, if it is CPU.
- `ProfileGPU` for one frame, saved into the document verbatim.
- Repeated at three points on the Amisbühl → Lehn route: on the launch slope
  (terrain and foliage close), in cruise (long view), and on final (ground
  detail plus HUD).

**Also record what the machine is doing**, since the symptom is thermal and not
frame time: power draw and fan state at idle, in the editor, and in the
standalone build, capped and uncapped. `powermetrics` gives package power on
Apple Silicon.

**Gate:** the table adds to the measured frame time within a stated
unaccounted share, the way the solver profile's does. A frame profile whose
stages sum to a third of the frame is not a measurement.

**What this level must be allowed to conclude:** that a level below it is
pointless. If the frame is 95% GPU in volumetric fog, Level 4's canopy
allocations are noise and should not be touched.

## Level 1 — the frames nobody asked for — **BUILT**

> **Done, with one correction to this section's own reasoning and one thing
> still owed.** The cap is in and gated. What is *not* measured is the symptom:
> package power before and after needs a running standalone build, which is
> Level 0's protocol and has not been run. Level 1 removed the cause the
> configuration indicted; whether that is *the* cause is still open.
>
> **The gate this section names was already green before it was written.**
> `determinism_tests` has a "render-rate independence" block that hashes 30 s
> of flight at 10, 30, 60, 120, 144 and 240 Hz plus two jittered rates: all
> eight identical. So no test needed writing for the claim that the clock does
> not care — it was built with the clock.
>
> **What nothing covered is the path by which a cap CAN reach the flight.**
> That block samples the controls as a function of simulation time, and its own
> comment says sampling per frame "would make the comparison meaningless" —
> correct for testing a clock. But `AParagliderPawn::Tick` samples input once
> per frame and applies it to every step that frame issues, so input sampling
> is the one channel from frame rate to handling. Measured, 30 s of flight
> against controls sampled every step:
>
> | frame rate | drift | airspeed drift |
> |---|---|---|
> | 240 Hz | **bit-identical** | 0 |
> | 144 Hz | **bit-identical** | 0 |
> | 120 Hz | 0.0007 m | 0.0000 m/s |
> | 60 Hz | 0.0776 m | 0.0015 m/s |
> | 30 Hz | 0.2292 m | 0.0045 m/s |
> | 20 Hz | 0.3792 m | 0.0075 m/s |
>
> **And the expected answer was wrong in a way worth keeping.** The derivation
> written first was "a cap at the simulation rate is exactly free, because one
> frame issues one step". Rates *above* the step rate are exactly free; the
> step rate itself is not, because 1/120 does not accumulate exactly, so
> occasionally one frame issues zero steps and the next issues two — and the
> second of those runs on a control value one step old. That is the whole
> 0.0007 m. It is 0.7 mm over 30 s against the 8 m a step covers, so the cap is
> sound at 120; what is not true is that it is bit-clean, and the record does
> not claim it.
>
> **Below the step rate the cost is real and monotonic** — 7.8 cm at 60 Hz —
> so the lower tiers buy their power with a handling change, now stated instead
> of assumed.
>
> **What landed:** `frameRateCapHz` and `verticalSync` on `GraphicsProfile`
> (Low 30 no vsync, Medium 60, High and Epic at the 120 Hz step rate);
> `ApplyGraphicsProfile` applying both; a `[SystemSettings] t.MaxFPS=120` /
> `r.VSync=1` startup baseline in `DefaultEngine.ini` so the first frames of a
> session are capped before a profile loads; the control-sampling table gated
> in `determinism_tests`; and the cap ordering, the pinning of the top tiers to
> the step rate, and "no tier is uncapped" gated structurally in
> `physics_tests`.
>
> **Found while here, not fixed, belongs to Level 4:** `DefaultEngine.ini` runs
> Unreal's own physics substepping at `MaxSubstepDeltaTime=0.008333` with
> `MaxSubsteps=8`, i.e. up to eight engine physics substeps per frame, in a
> project whose flight dynamics are not Unreal's. Whether anything depends on
> it — terrain collision, the rollout — is unmeasured, so it is a suspect and
> not yet a saving.



The cheapest suspect, and the one the configuration already indicts.

- **Cap the frame rate and offer vsync.** A simulator that renders 300 fps
  against a 120 Hz sim delivers nothing a pilot can perceive and costs the
  whole difference in power. The fixed-step clock already decouples this
  correctly — `ParagliderSolverClock` owns the accumulator, clamps hitches at
  0.25 s, and `InterpolateGliderRigSnapshot` is already applied every frame at
  `InterpolationAlpha`, so the render rate can move without touching the
  physics. **This is the reason this level is cheap: the hard part is already
  built.**
- The cap belongs with the graphics profiles (`ApplyGraphicsProfile`,
  `Scalability::SetQualityLevels`), which already exist and are already
  cycleable, rather than as a loose console variable.

**Instrument:** package power and `stat unit` at cap 60, cap 120, and uncapped,
from Level 0's three scene points.

**Gate:** *simulation output is bit-identical across frame rates.* This is the
one that matters and it is testable without the engine: the fixed-step clock's
step count is derived from delivered time, so a run fed 60 fps deltas and one
fed 144 fps deltas must produce the same state to the bit for the same total
elapsed time. `determinism_tests` is where that belongs, and it is a real test
rather than a timing assertion.

**Risk stated up front:** if Level 0 shows the frame is already GPU-bound at a
low frame rate, this level saves power but not the symptom, and Level 4 or 5
owns it instead.

## Level 2 — the atmosphere sample, because it is what the wind work grows

Out of order relative to cost — it may well be small — and deliberately so.
Every other level is about the game we have; this one is about whether the game
we are about to build fits.

- **Instrument:** a new `parapenting_air_profile` binary beside
  `parapenting_solver_profile`, same rules: asserts nothing, not in
  `check-build.sh`. It times `AtmosphereModel::Sample` and `SampleCanopy`
  across the presets that exist — chill, ridge, localized rotor, rotor
  everywhere — and across altitude bands, because thermals and rotor are not
  uniformly priced.
- **The number that matters is per step, against 8333 µs**, stated beside the
  solver's 540 so the two are comparable in one place.
- **Then the forward-looking one, which is the point of the level:** what a
  richer wind field would cost per sample, measured by scaling the existing
  model's harmonic/eddy count rather than by guessing. If doubling the
  structure doubles the cost, the wind work has a stated price per feature
  before a line of it is written.

**Gate:** `FRAME_PROFILE.md` and `SOLVER_PROFILE.md` between them account for
the atmosphere sample explicitly, so that it can never again be a stage that no
table contains.

## Level 3 — the terrain render mesh

64 sections, ~320k triangles, no LOD, drawn at full density from 10 km away.

Options, in the order they should be measured rather than the order they are
attractive:

1. **Distance-based tier selection.** The layout is generated
   (`TerrainRenderLayout.h`), so `cellsPerTile` can vary per tile by distance
   from the route's flight corridor rather than being one constant. Cheapest to
   build, no asset pipeline.
2. **Static mesh conversion with a real LOD chain**, which is what the engine
   is good at and what a procedural component cannot do.
3. **Nanite**, which is the obvious answer on paper and needs measuring on
   Apple Silicon specifically before it is believed.

**The gate is the one that protects the physics**, and it is non-negotiable:
**the terrain is a measurement, and only its rendering may change.**
`TerrainModel::HeightM` feeds collision, airflow and the survey provenance;
`terrain_survey_tests` and the route anchors must be untouched and must still
pass unmodified. A rendering change that moves a sampled height is a
correctness bug, not a performance trade.

**Second gate:** the visual QA route pass in `VISUAL_QA.md`, because a terrain
LOD that pops on approach is a worse game than a hot one.

## Level 4 — the per-frame game thread

Only if Level 0 says the frame is CPU-bound. In suspicion order, all from
reading `ParagliderPawn.cpp`:

- **`UpdateCanopyMesh` allocates five `TArray`s per frame** — vertices,
  normals, UVs, colours, tangents for 846 vertices — and calls
  `UpdateMeshSection`. The arrays are the same size every frame and should be
  members, filled in place. This is a mechanical change with a mechanical gate.
- **The suspension mesh and the air motes** rebuild on the same pattern.
- **`CaptureGliderRigSnapshot` runs per fixed step, not per frame** — correct,
  since it is what interpolation needs, but it means its cost is multiplied by
  the step count and it has never been in a table.
- **The HUD** issues a large number of individual `DrawText` calls per frame.

**Gate:** allocation count per frame, which is a structural property and can be
asserted, rather than a millisecond count, which cannot. **And a null gate that
is easy to skip and should not be:** the pilot rig, canopy and lines must be
visually identical before and after — this is refactoring, and any visible
difference means it was not.

## Level 5 — what the GPU is actually spending it on

`DefaultScalability.ini` already defines four tiers across view distance,
shadows, volumetric fog, volumetric clouds and foliage density, and
`ApplyGraphicsProfile` already applies them. **What is missing is a measurement
of what each tier buys, and therefore a defensible default.** Today the default
is whatever the engine picks.

- **Instrument:** Level 0's protocol, run once per tier, at the three scene
  points. One table, five columns.
- The suspects worth isolating on Apple Silicon are volumetric fog grid size,
  volumetric cloud ray samples, virtual shadow maps, and Lumen — none of which
  the project has ever configured explicitly, so all four are at engine
  defaults.
- **The output is a chosen default profile with the number that chose it**, and
  the tier table published in `FRAME_PROFILE.md`.

## Level 6 — the headroom statement, which is the actual deliverable

The reason for all of the above is the next physics, so the plan ends with the
budget rather than with a saving.

Written as one table, per fixed step at 120 Hz against 8333 µs, once Levels 0–5
have filled it in:

| stage | µs/step | measured by |
|---|---|---|
| coupled solver, full tier | 540 | `SOLVER_PROFILE.md` |
| coupled solver, reduced tier | 231 | `SOLVER_LOD.md` |
| atmosphere sample | — | Level 2 |
| rig snapshot | — | Level 4 |
| **budget remaining for wind and lift** | — | this table |

**The gate on the wind and lift work is then a number rather than a hope:** its
per-step cost is stated before it is built, measured after, and it fits inside
what this table leaves, or the schedule amortises it the way
`aerodynamicsInterval` already amortises the aerodynamics 12:1.

---

## What this plan explicitly does not do

- **It does not optimise the flight solver.** 6.5% of one core, measured twice.
  Nothing here touches it.
- **It does not spend the coupling iterations.** `SOLVER_LOD.md` measured 3 → 2
  moving the asymmetric collapse peak from 0.648 to 0.691 — 6.5% on the number
  a pilot is judged on — for 21% of a step that is already 15× faster than real
  time. That trade stays refused.
- **It does not touch anything below the membrane's 1.7%.** Pressure, collapse
  and rigid motion together are 0.3% of a step.
- **It does not make the reduced tier the default.** Every published number and
  every Level 9 calibration gate is measured on the full schedule, and
  `SOLVER_LOD.md` states plainly that the reduced tier is not the full solver
  during transients.
- **It does not add a timing assertion to `check-build.sh`.**

## Order of work

Level 0 first and alone, because it can invalidate any of the others. Then
Level 1, because it is one setting and the configuration already indicts it.
Then whichever of 3, 4 or 5 Level 0 points at — **not all three**. Level 2 can
run in parallel with any of them, since it touches nothing the others do, and
it must land before the wind work starts rather than alongside it.

## The one number that would close this document

Package power, on the same route, at the same visual quality, before and after:
the symptom that opened it was thermal, and frame time is a proxy for it rather
than the thing itself. Level 0 records it so that the last line of this plan
can be a comparison rather than an argument.
