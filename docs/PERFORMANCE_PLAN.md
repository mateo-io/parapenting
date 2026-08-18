# Performance: the frame, not the solver

The symptom was fans, on a fast machine, with wind and lift still to build. This
is the plan for the headroom, written the way the physics ladder is written:
levels, gates and tests, one knob at a time.

**Rewritten 2026-08-18, after the measuring level ran.** The first version was a
ladder of suspects ordered by suspicion. Two of them were wrong and the
measurement said so, so the order below is now the measured one. What the first
version got wrong is kept at the bottom, because the shape of the error is worth
more than the tidy document would be.

## What is measured, and it is the whole basis of the order below

| | | source |
|---|---|---|
| **GPU** | **7.83 ms — 93% of the frame** | `FRAME_PROFILE.md` |
| ├ TemporalSuperResolution | **3.48 ms — 44% of GPU** | |
| └ everything else | 4.35 ms | |
| **Game thread** | **5.07 ms** | `FRAME_PROFILE.md` |
| ├ TickActors | 3.68 ms | |
| └ of which the flight solver | ≤0.54 ms | `SOLVER_PROFILE.md` |
| Frame | 8.41 ms, 118.9 fps at a 120 cap | |

**The frame is GPU-bound**, and the flight solver — the thing this project is —
is 6.5% of one core. Nothing in this plan touches it.

## The order, and why each position is held by a number

1. ~~**[L1] Fix the capture harness.**~~ **Done** — `Tools/frame-capture.sh`,
   reproducible to 1% on frame and GPU.
2. ~~**[L2] TSR and the GPU feature tiers.**~~ **Measured**: TAA −19% of GPU,
   TAA+67% −22%, nothing else above 3%. Not shipped — needs a look at the
   lines in motion.
3. **[L3] The actor tick that is not the solver.** ~3.1 ms of the game thread.
   **NEXT**, and now the larger of the two remaining costs: with the GPU at
   6.26 ms under L2's best row, the 5.07 ms game thread stops being second.
4. **[L4] The atmosphere sample.** Unmeasured, and it is what the wind work
   grows. Must land *before* that work, not alongside it.

**Closed:** the frame cap (built, and measured to buy 1.4 fps — right, but not
the cause). **Cancelled:** the terrain render mesh — the entire base pass is
0.60 ms.

## How this plan is allowed to be wrong

- **Measure before cutting.** A level that cannot state its instrument does not
  start. Two levels of the first version were written without one and both were
  wrong.
- **One knob per row**, `SOLVER_LOD.md`'s sweep as the model: one change off the
  reference, the same signatures, cost and consequence in one table.
- **No wall-clock assertion in `check-build.sh`.** Wall clock is a property of
  the machine. Profiles assert nothing and live in their own binaries. What
  *can* be gated is structure — that no tier is uncapped, that terrain heights
  are unchanged, that a frame allocates no more than it did.
- **Visual changes are gated by `VISUAL_QA.md`, not by opinion.** L2 and L3 both
  change what the pilot sees.

---

## L1 — a capture harness that gives the same answer twice — **DONE**

> `Tools/frame-capture.sh`. Reproduces frame and GPU to **1%**, per-pass to
> **11%**; scene spread down from 97% to 10–13% by aligning to the flight
> instead of freezing it. Variance and method in `FRAME_PROFILE.md`.
>
> **The real cause of Level 0's failed freezes was not the engine lifecycle —
> it was that `-ExecCmds` separates on commas, not semicolons.** Everything
> after the first `;` was swallowed as an argument. The first diagnosis written
> down blamed world-versus-cvar timing, which was plausible, tidy and wrong.
>
> Validation row: TAA instead of TSR is **−1.5 ms of GPU (18%)**, which is the
> harness detecting the change it exists to measure.

Level 0 produced a frame profile it could stand behind and **two comparisons it
could not**, both for instrument reasons rather than engineering ones:

- **The scene moves.** The aircraft is flying, so two captures compare different
  parts of the route. Early flight near the launch slope costs 15–28 ms of GPU
  against open air's 7.8 — three times the total saving L2 is chasing. Aligning
  by frame index reported the scene; aligning by wall time did not fix it,
  because level load time varies and slides the flight under the window.
- **The window loses foreground and the engine throttles.** CPU swinging 7% to
  137%, one frame at 1013 ms against a 7.5 ms median.

**What the harness must do**, and each of these is one of the two failures:

1. **Freeze the flight** so the view is identical between runs (`slomo`, or a
   fixed camera), and *verify* it froze rather than assume — the first attempt
   used `pause`, which did not take, and `TickActors` at 3.68 ms was the
   evidence.
2. **Defeat the background throttle** (`t.IdleWhenNotForeground 0`) so an
   unfocused capture is still a capture.
3. **Settle before capturing**, so shader compilation and level load are not in
   the average.
4. **Report by column name**, not column index — the CSV's layout changes
   between runs, which silently moved `FrameTime` between two captures during
   Level 0.

**Gate:** the same configuration, captured twice, agrees to within a stated
spread. A harness that cannot reproduce itself cannot measure a 10% change.

**Deliverable:** `Tools/frame-capture.sh`, and its variance stated in
`FRAME_PROFILE.md`.

## L2 — TSR, and what the GPU is actually spending it on — **MEASURED, NOT SHIPPED**

> Sweep in `FRAME_PROFILE.md`. **TAA is −1.53 ms of GPU (19%), TAA at 67%
> screen is −1.79 (22%)**, and everything else on the list is worth 3% or
> nothing.
>
> **The level's own premise needed correcting: TSR is 44% of the GPU frame but
> removing it returns 19%.** A pass's cost is not the recoverable cost.
>
> **No change shipped.** At a 120 cap none of this moves the frame rate — it
> moves GPU occupancy from 93% to 74%, which is the thermal symptom and the
> right quantity. But the wing hangs on 254 m of 1 mm line, which is the
> sub-pixel geometry a temporal upscaler exists to hold together, so the trade
> needs eyes on the lines in motion. That is `VISUAL_QA.md`'s call, and then it
> is one line in `GraphicsProfile.cpp`, per tier.

3.48 ms of a 7.83 ms frame, in an upscaler running at **100% screen
percentage** — reconstructing a frame from a frame the same size. More than
base pass, shadows, lighting, SSAO, fog and clouds combined (2.65 ms).

**The sweep**, one knob per row, on L1's harness:

| row | knob |
|---|---|
| reference | TSR, 100% screen percentage |
| | TAA instead of TSR |
| | FXAA instead of TSR |
| | TSR at 67% screen percentage — the job an upscaler exists for |
| | TSR at 77% |
| | volumetric cloud shadow off (0.30 ms) |
| | SSAO off (0.28 ms) |

**The knob that is not like the others** is screen percentage: it cuts the base
pass, shadows and lighting at the same time, and it is what TSR is *for*. A
row that saves 3.5 ms by making the image worse is not automatically a win, and
this is where `VISUAL_QA.md` is the gate rather than a formality.

**Then the tiers get their numbers.** `DefaultScalability.ini` already defines
four, and `ApplyGraphicsProfile` already applies them; what has never existed is
a measurement of what each buys, and therefore a defensible default.

## L3 — the actor tick that is not the solver

5.07 ms of game thread, `TickActors` 3.68, at most 0.54 of it the flight solver.
The rest is per-frame work in `ParagliderPawn`, all found by reading and none of
it yet attributed:

- `UpdateCanopyMesh` allocates five `TArray`s per frame for 846 vertices and
  calls `UpdateMeshSection`; the suspension mesh and air motes repeat the
  pattern. The arrays are the same size every frame and should be members.
- `CaptureGliderRigSnapshot` runs per fixed **step**, so its cost multiplies by
  the step count and has never been in a table.
- The HUD issues many individual `DrawText` calls.
- Found in Level 1 and still unowned: Unreal's own physics substepping runs up
  to 8 substeps per frame in a project whose flight dynamics are not Unreal's.

**Instrument first:** attribute the 3.68 ms between these before touching any of
them. **Gate:** allocations per frame, which is structural and assertable, plus
an unchanged rig visually — this is refactoring, and any visible difference
means it was not.

## L4 — the atmosphere sample, before the wind work

`AirModel.SampleCanopy` runs once per fixed step from the pawn, and
`AtmosphereModel.cpp` is 25 KB of gusts, thermals, ridge lift, slope circulation
and rotor. `Tests/SolverProfile.cpp` constructs a `CoupledAtmosphere` and never
calls it, so **this stage has never appeared in any table** — and it is exactly
what wind and lift will grow.

- **Instrument:** `parapenting_air_profile`, beside `parapenting_solver_profile`,
  asserting nothing. Per sample and per step, across the weather presets and
  altitude bands, stated beside the solver's 540 µs so the two are comparable.
- **Then the forward-looking row:** what richer wind costs, measured by scaling
  the existing model's structure rather than guessed, so the wind work has a
  price per feature before it is written.

**Gate:** the budget table below has no blank rows left in it.

| stage | µs/step of 8333 | measured by |
|---|---|---|
| coupled solver, full tier | 540 | `SOLVER_PROFILE.md` |
| coupled solver, reduced tier | 231 | `SOLVER_LOD.md` |
| atmosphere sample | — | L4 |
| rig snapshot | — | L3 |
| **remaining for wind and lift** | — | this table |

---

## What this plan refuses

- **The flight solver.** 6.5% of one core, measured twice.
- **The coupling iterations.** 3 → 2 moves the asymmetric collapse peak from
  0.648 to 0.691 — 6.5% on the number a pilot is judged on — for 21% of a step
  that is already 15× faster than real time.
- **Anything below the membrane's 1.7%.** Pressure, collapse and rigid motion
  together are 0.3% of a step.
- **The reduced tier as default.** Every published number and every Level 9
  calibration gate is measured on the full schedule.
- **A timing assertion in `check-build.sh`.**

## What the first version of this plan got wrong

Kept because the plan's own rule is that a level states its instrument first,
and these are what happens when it does not.

- **"The terrain is 64 procedural sections with no LOD, drawn at every
  distance."** True, and irrelevant: the whole base pass is 0.60 ms. The
  argument went from a structural fact to a performance conclusion with no
  measurement in between. It was a whole level, and it is cancelled.
- **"There is no frame-rate cap; an uncapped renderer will produce as many
  frames as the GPU can make."** True, and it was still nearly free to fix:
  uncapped ran 119.6 fps against the cap's 118.2, because the GPU was already
  the limit at ~128. The cap is right and is not the cause of the symptom.
- **What no version of the plan predicted** is the thing that actually dominates
  the frame: an anti-aliasing pass at 44% of the GPU. Nothing about it is
  visible from reading the configuration, which is the argument for L1 and the
  whole reason Level 0 blocks everything.
