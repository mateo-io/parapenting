# Frame profile

Performance plan Level 0. `SOLVER_PROFILE.md` measured the flight solver;
this measures everything else, which had never been measured at all.

```sh
# The capture, from the project root. Writes into
# ~/Library/Application Support/Epic/UnrealEngine/5.8/Saved/Profiling/CSV/
"/Users/Shared/Epic Games/UE_5.8/Engine/Binaries/Mac/UnrealEditor.app/Contents/MacOS/UnrealEditor" \
  "$PWD/Parapenting.uproject" -game -windowed -resx=1280 -resy=720 \
  -csvCaptureFrames=2400 -csvGpuStats -ExecCmds="t.MaxFPS 120"
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
