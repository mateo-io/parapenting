# Atmosphere profile

Performance plan L4. `SOLVER_PROFILE.md` measured the flight solver,
`FRAME_PROFILE.md` the frame; between them they never contained the atmosphere
sample. `Tests/SolverProfile.cpp` constructs a `CoupledAtmosphere` and never
calls it, so this stage had no number at all — and it is the one the wind and
lift work grows.

```sh
cmake --build build/tests --target parapenting_air_profile
./build/tests/parapenting_air_profile
```

Asserts nothing and is not in `Tools/check-build.sh`, like the solver profile
beside it. Apple M1 Max, Release, measured 2026-08-19.

## What the pawn actually does

One `SampleCanopy` per fixed step, which is **three `Sample` calls** — centre
and both wing tips. Everything else (`SampleCloudField`, the briefing's
launch/cruise/landing samples) is per-event, and the airflow visualisation's
probe grid only runs when it is toggled on.

## The number

| | ns/sample | µs/step (×3) | share of the 8333 µs step |
|---|---|---|---|
| STILL AIR LAB | 47 | 0.14 | 0.00% |
| EVENING DRAINAGE | 816 | 2.45 | 0.03% |
| VALLEY BREEZE | 831 | 2.49 | 0.03% |
| ACTIVE THERMAL DAY | 835 | 2.50 | 0.03% |
| FOEHN / STRONG ROTOR | 1151 | 3.45 | 0.04% |

**The atmosphere is 0.03% of a step.** It was never a cost; it simply had never
been counted, which is a different thing and worth the level it took to find out.

## The sample is 90% terrain query

| component | ns/call |
|---|---|
| `TerrainModel::LeeRotorPotential` | 347 |
| `TerrainModel::RidgeExposure` | 174 |
| `TerrainModel::Normal` | 165 |
| `TerrainModel::HeightM` | 39 (called twice) |
| **= terrain in one sample** | **764 of 846 — 90%** |

Position barely matters — 838 ns in mid valley, 837 low over the ground, 841
above the thermal top, 665 off the terrain grid entirely. So the cost is the
queries themselves rather than what the terrain happens to be doing there.

## The row that was wrong, and what replaced it

The first version priced "one more piece of weather" by regressing sample cost
against the number of active weather volumes across presets, and reported
**204 ns per volume**. Its own table refuted it: still air carries zero volumes
at 47 ns, evening drainage carries zero at 816, and the four-volume thermal day
is *cheaper* than the two-volume valley breeze.

**Volume count does not order the rows. Weather mode does**, because the mode
decides which terrain queries run. The regression had one variable in it and
two causes behind it.

## What this says to the wind and lift work

**More weather structure is free.** Doubling the number of thermals, rotors or
sink volumes moves nothing measurable, because the sample is dominated by
terrain queries that run regardless.

**More sample points is what scales**, linearly, and that is the axis the wind
work will actually move along — a per-section wind field instead of centre and
tips:

| sampling | µs/step | share |
|---|---|---|
| 3 — centre and tips, what ships today | 2.59 | 0.03% |
| 16 — one per spanwise strip | 13.8 | 0.17% |
| 45 — one per VSM section | 38.9 | 0.47% |
| 135 — one per VSM section, with tips | 116.7 | 1.40% |

**Even the most extravagant of those is 1.4% of a step.** If per-section wind
turns out to be worth having physically, its cost is not the reason to refuse
it — and if it ships alongside the coupled solver becoming the flight model,
the pair is under 8%.

## The budget, with no blank rows left

| stage | µs/step of 8333 | share | measured by |
|---|---|---|---|
| atmosphere sample, as shipped | 2.6 | 0.03% | this document |
| flight model the game runs (`ParagliderDynamics`) | ~13 | 0.16% | `FRAME_PROFILE.md` L3 |
| rig snapshot | 0.2 | 0.00% | `FRAME_PROFILE.md` L3 |
| canopy render mesh (per frame, not per step) | 138 | — | `FRAME_PROFILE.md` L3 |
| coupled solver, full tier — **not the flight model yet** | 540 | 6.5% | `SOLVER_PROFILE.md` |
| coupled solver, reduced tier | 231 | 2.8% | `SOLVER_LOD.md` |
| **remaining for wind and lift** | **~7800** | **94%** | this table |

The headroom question that opened the performance plan is answered: **the
simulation side of the step is essentially empty.** What was hot was never
physics — it was a cell relaxation solved 846 times a frame in the render path
(L3) and a GPU frame dominated by an upscaler (L2). Wind and lift can be
built against a step that is 94% unspent.
