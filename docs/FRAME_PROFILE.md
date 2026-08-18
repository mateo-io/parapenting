# Frame profile

Performance plan Level 0. `SOLVER_PROFILE.md` measured the flight solver;
this measures everything else, which had never been measured at all.

```sh
Tools/frame-capture.sh reference                        # one row
Tools/frame-capture.sh "TAA" r.AntiAliasingMethod=2     # one knob off it
```

Like the solver profile, this asserts nothing and is not part of
`Tools/check-build.sh`. Wall clock is a property of the machine.

## Machine and configuration, including what is compromised about it

Apple M1 Max, macOS 15 (Darwin 25.3.0), measured 2026-08-18 at commit
`82a633f`. 1280×720 windowed, EPIC graphics profile, frame cap 120.

**Run through the editor in `-game` mode, not a standalone build**, which the
performance plan asks for and does not get here: `Binaries/Mac/Parapenting.app`
is from 2026-07-30 and predates three weeks of physics work, so it would have
measured the wrong game. The editor process carries overhead the shipped game
would not. **Every number below is therefore an upper bound on the game
thread and a fair measure of the GPU**, which is what the conclusions rest on.

**Amisbühl → Lehn, from launch, no pilot input.** Four independent captures.

## The frame

| | ms | note |
|---|---|---|
| **FrameTime** | **8.41** | 118.9 fps against a 120 cap |
| GameThread | 5.07 | |
| **GPUTime** | **7.83** | 93% of the frame |
| RenderThread | 3.67 | rises to 8.4 when it is waiting on the GPU |
| RHIThread | 2.17 | |

Reproducible: GPU 7.81, 7.83, 7.84, 7.97 across four captures; game thread
5.00–5.07.

## Where the GPU goes, and it is not the scene

| pass | ms | share of GPU |
|---|---|---|
| **TemporalSuperResolution** | **3.48** | **44%** |
| Basepass | 0.60 | 8% |
| ShadowDepths | 0.54 | 7% |
| RenderDeferredLighting | 0.41 | 5% |
| Postprocessing | 0.31 | 4% |
| VolumetricCloudShadow | 0.30 | 4% |
| SSAO | 0.28 | 4% |
| ShadowProjection | 0.22 | 3% |
| Lights | 0.16 | 2% |
| ReflectionEnvironment | 0.14 | 2% |

## Where the game thread goes

| | ms |
|---|---|
| Exclusive/GameThread/TickActors | 3.68 |
| everything else | 1.39 |

At 120 fps the fixed-step clock issues one 120 Hz step per frame, so **at most
0.54 ms of that 3.68 is the flight solver** — the figure `SOLVER_PROFILE.md`
measures. The remaining ~3.1 ms is actor tick work: the canopy and suspension
mesh rebuilds, the pilot skeleton, the air motes, the HUD.

## Four things this says

**1. The frame is GPU-bound, and the frame cap was not the cause of the fans.**
GPU 7.83 ms is a ~128 fps ceiling, so the renderer was never running away.
Measured directly rather than inferred — same scene, same session:

| | fps | GPU ms |
|---|---|---|
| uncapped | 119.6 | 7.79 |
| capped at 120 | 118.2 | 7.84 |

**The cap bought 1.4 fps.** Level 1 is still right — it costs nothing
measurable, it protects machines that are not this one, and it removed a
genuinely uncapped configuration — but **it is not why the fans spin.** They
spin because the GPU is ~93% busy every frame at 120 fps, and that is a
per-frame cost, not a frame-count problem.

**2. Temporal Super Resolution is 44% of the GPU frame.** It is an upscaler,
and it is running at 100% screen percentage — reconstructing a frame from a
frame the same size. Its 3.48 ms is more than the base pass, shadows, lighting,
SSAO, fog and clouds put together (2.65 ms). This is the single largest lever
in the project by a wide margin and it is one setting.

**3. The terrain is not a performance problem, and Level 3 should be dropped.**
The plan reasoned that 64 procedural sections at ~320k triangles with no LOD
would be expensive. The entire base pass is **0.60 ms**, 8% of the GPU frame.
Nothing about terrain LOD, static mesh conversion or Nanite would return
meaningful time. **This is the level the plan said Level 0 must be allowed to
cancel, and it cancels it.**

**4. The game thread is real but second, and it is not the physics.** 5.07 ms
against a 8.33 ms budget at 120 Hz, of which at most 0.54 ms is the solver.
Level 4 keeps its place behind Level 5.

## What this measurement could not do, stated rather than omitted

**The power number is still owed, and it is the one the symptom is about.**
`powermetrics` needs root, so it is not something this profile could run:

```sh
sudo powermetrics --samplers cpu_power,gpu_power -i 1000 -n 30
```

Process CPU was 250–270% (2.5–2.7 cores) at the 120 cap, sampled with `ps`.
That is a proxy and not power.

**The 60 fps comparison failed twice, and both failures are instrument
failures worth recording.**

- *Flight-phase misalignment.* The first attempt compared capture windows by
  frame index. The aircraft is flying, so different frame rates put the same
  window at different points of the route — early flight near the launch slope
  costs 15–28 ms of GPU, open air costs 7.8. It reported the scene, not the
  cap. Aligning by accumulated wall time did not fix it either, because level
  load time varies between runs and shifts the flight underneath the window.
- *Background throttling.* Freezing the aircraft with `slomo 0.001` fixed the
  scene, but the launched window does not reliably hold foreground, and an
  unfocused run throttles: CPU samples swung 7%–137% and one frame took 1013 ms
  against a 7.5 ms median. Any average over that is contaminated.

**So "what does capping at 60 save" remains unmeasured**, and the plan should
not spend it until the harness holds a fixed scene in the foreground. Note that
`GraphicsProfile.cpp` already ships 60 on the MEDIUM tier and 30 on LOW, chosen
on the control-sampling measurement in `determinism_tests` rather than on this.

**Also not done:** the plan's three scene points (launch slope, cruise, final)
were not captured separately, and the numbers above are a launch-to-cruise
average. Given how much the slope-versus-air spread turned out to matter —
7.8 ms against 15–28 — that split is now the first thing a second pass should
do.

## What to do next, in the order the numbers put it

1. **Level 5, and specifically TSR.** 3.48 ms of 7.83. Either drop to TAA,
   or lower TSR's quality, or — the option that fits an upscaler — render at a
   lower screen percentage and let it do the job it exists for, which would cut
   the base pass, shadows and lighting at the same time.
2. **Level 4**, the ~3.1 ms of actor tick that is not the solver.
3. **Level 2**, the atmosphere sample, before the wind work rather than with it.
4. **Level 3 is closed** by point 3 above.


---

# The harness (plan L1)

The profile above was taken by hand and produced two comparisons it could not
stand behind. `Tools/frame-capture.sh` is what replaced that, and it is a
level of the plan rather than a convenience because **an A/B is unmeasurable
until it is repeatable**.

## Repeatability, which is the gate

Same configuration, two runs:

| | frame | GPU | TSR |
|---|---|---|---|
| reference A | 8.66 ms | 8.10 ms | 3.70 ms |
| reference B | 8.57 ms | 8.01 ms | 3.31 ms |
| spread | **1.0%** | **1.1%** | **11%** |

**Frame and GPU reproduce to about 1%, per-pass numbers to about 11%.** So a
row that moves the GPU by less than ~2% is not a result, and a per-pass
attribution that moves by less than ~20% is not either. Stated because the next
level is a sweep and every row of it will be read against these two numbers.

## What it does, and why each part is one of Level 0's failures

- **Aligns to the flight rather than freezing it.** The simulation is
  deterministic and locked to real time, so the same seconds after the flight
  starts is the same aircraft in the same place at any frame rate. The capture
  begins at *engine* start and level load varies, so the harness finds the
  flight in the capture itself — the first frame after which two continuous
  seconds pass with nothing over 50 ms — and measures from there. Scene spread
  fell from **97% to 10–13%**.
- **Defeats the background throttle** with `t.IdleWhenNotForeground 0`, which is
  what made Level 0's unfocused runs swing between 7% and 137% of CPU.
- **Reads the CSV by column name**, because the column layout changes between
  captures — `FrameTime` moved 27 columns between two of Level 0's runs.
- **Reports the scene spread**, so a row that measured the route instead of the
  knob says so.

## The bug that invalidated Level 0's freeze attempts, and the wrong diagnosis

Both `pause` and `slomo 0.001` appeared to be ignored, and the first
explanation written down was that `-ExecCmds` runs before a world exists, so
world commands are dropped while cvars survive. **That was plausible and it was
wrong.**

**`-ExecCmds` separates on commas, not semicolons.** The semicolon-joined
string was executed as a single command, so the leading cvar swallowed the rest
as its argument and stopped parsing at the first `;`. That is also why
`t.MaxFPS 120; slomo 0.001` still capped at 120 — the number parsed — which is
what made the failure invisible: the cap worked, so the string looked fine, and
the blame went to the engine's lifecycle.

Caught by reading the log for the command rather than reasoning about it. With
commas, the same mechanism works first time.

## The first sweep row, as validation

Not L2 — one row, to prove the harness detects the change it was built for:

| | frame | GPU | top pass |
|---|---|---|---|
| reference (TSR, 100%) | 8.57–8.66 ms | 8.01–8.10 | TSR 3.31–3.70 |
| `r.AntiAliasingMethod 2` (TAA) | 8.35 ms | **6.58** | Unaccounted 0.85 |

**−1.5 ms of GPU, 18%**, and TSR leaves the pass list entirely. Against a 1.1%
repeatability that is a result rather than noise. It is one run of one row and
the image has not been looked at, so it is L2's starting point and not L2's
answer.


---

# L2 — the GPU sweep

One knob per row on `Tools/frame-capture.sh`, reference measured twice at the
start and end of the sweep. GPU is the column that matters; frame time is
capped at 120 and barely moves, which is itself the point (below).

| row | GPU ms | vs reference | top pass |
|---|---|---|---|
| **reference** (TSR, 100%) | 8.05 | — | TSR 3.64 |
| reference, repeated at the end | 8.06 | +0.1% | TSR 3.65 |
| **TAA + 67% screen** | **6.26** | **−1.79 (−22%)** | Postprocessing 0.73 |
| FXAA | 6.49 | −1.56 (−19%) | Unaccounted 0.77 |
| **TAA** | **6.52** | **−1.53 (−19%)** | Unaccounted 0.77 |
| TSR at 67% screen | 7.00 | −1.05 (−13%) | TSR 1.90 |
| TSR at 77% screen | 7.53 | −0.52 (−6%) | TSR 2.46 |
| no SSAO | 7.83 | −0.22 (−3%) | TSR 3.66 |
| no volumetric cloud | 7.84 | −0.21 (−3%) | TSR 3.65 |
| `r.TSR.History.ScreenPercentage 100` | 8.05 | 0.00 | TSR 3.66 |

## The correction this sweep makes to its own premise

**TSR is 44% of the GPU frame and removing it returns 19%, not 44%.** The pass
costs 3.64 ms; deleting it saves 1.53. Whatever replaces it — TAA, FXAA — does
work of its own, and FXAA landing within 0.03 ms of TAA says that remainder is
not the anti-aliasing method either. **A pass's cost is not the recoverable
cost**, and the plan promoted this level on the assumption that it was.

The saving is real and it is the largest single one available. It is just
two thirds smaller than the attribution implied.

## Three things the rows say

**1. Switching the AA method beats upscaling with TSR.** TAA at full resolution
(6.52) is cheaper than TSR at 67% (7.00), and TSR at 77% barely pays (−0.52).
Stacking them is worth only another 0.26 ms — TAA + 67% at 6.26 against TAA's
6.52 — so the two levers are not additive and the resolution one is nearly
spent once the method changes.

**2. Nothing else on the list is worth having.** SSAO and volumetric cloud are
0.22 and 0.21 ms — each within a rounding error of the pass cost the profile
attributed to them, which is a small confirmation that the attribution is sound
where the pass is a leaf. `r.TSR.History.ScreenPercentage` is inert.

**3. None of it changes the frame rate, and that is the point.** Every row sits
between 8.4 and 8.8 ms because the 120 cap binds. What moves is **GPU
occupancy: 8.05 of 8.61 ms is 93% busy; 6.26 of 8.47 is 74%.** The symptom
that started this plan was thermal, so occupancy is the quantity, and fps was
never going to show it.

## Repeatability, restated more precisely than L1 did

The reference measured 8.05 at the start of the sweep and 8.06 at the end —
**0.1%**, better than the 1.1% the L1 pair suggested. But the *within-window*
scene spread ran 20–84% across rows, and one row had to be thrown away and
repeated: volumetric cloud first read 8.19 ms, i.e. slower than the reference,
at a spread of 352%.

**So the two numbers measure different things and only one of them is the
error bar.** Run-to-run variance of the mean is ~0.1–1%. Within-window spread
is the scene changing during the window, which inflates no average but does
signal a row that wandered somewhere unrepresentative. Any row over ~100%
spread should be repeated before it is read, which is the rule this sweep
adopted after the cloud row.

## What is NOT decided here, and why

**The image has not been judged, and this level does not get to decide it.**
`VISUAL_QA.md` is the gate, and there is a specific reason to be careful on
this wing rather than a general one: the aircraft is suspended on **254 m of
1 mm line**, which is exactly the sub-pixel geometry temporal upscalers exist
to hold together and spatial anti-aliasing does not. A 19% GPU saving that
makes the risers shimmer in flight is not obviously a good trade on a
simulator whose whole subject is the wing.

So this level ships **no change**. It ships the numbers, and the recommendation
that the comparison worth looking at is **TAA against TSR, in motion, on the
lines** — at which point the choice is one line in `GraphicsProfile.cpp` and
can be per tier rather than global.
