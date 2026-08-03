# Solver levels of detail

Level 10, strand 2. `SOLVER_PROFILE.md` said where the time goes; this says how
much of it can be given back, and what it costs in answers.

```bash
Intermediate/PhysicsTests/parapenting_solver_lod
```

Like the profile, it asserts nothing — it is the sweep the tolerances are
derived from. What *is* gated is the tier itself, in `coupled_tests`, against
the full solver, on every run.

## The sweep

Apple M1 Max, Release, commit at time of writing. Each row moves **one** knob
off the reference schedule and flies two signatures: settled hands-up trim, and
a 4 m/s downdraught over the left half — the gust that folds this wing.

| schedule | v m/s | α | fold | residual N | µs/step | saves |
|---|---|---|---|---|---|---|
| **reference** 120 / 3 / 600 / 1 | 10.738 | 4.70° | 0.648 | 0.140 | 522 | — |
| suspensionIterations 80 | 10.738 | 4.70° | 0.648 | 0.129 | 418 | 20% |
| suspensionIterations 60 | 10.738 | 4.70° | 0.648 | 0.155 | 365 | 30% |
| **suspensionIterations 40** | 10.738 | 4.70° | 0.648 | 0.198 | 312 | 40% |
| suspensionIterations 30 | 10.738 | 4.70° | 0.648 | 0.257 | 286 | 45% |
| suspensionIterations 20 | 10.738 | 4.70° | 0.648 | 0.384 | 267 | 49% |
| suspensionIterations 10 | 10.738 | 4.70° | 0.648 | 0.763 | 231 | 56% |
| couplingIterations 2 | 10.718 | 4.73° | **0.691** | 0.113 | 412 | 21% |
| couplingIterations 1 | 10.613 | 4.91° | **0.760** | 0.013 | 270 | 48% |
| frozenSolveIterations 300 | 10.738 | 4.70° | 0.648 | 0.140 | 521 | 0% |
| frozenSolveIterations 40 | 10.737 | 4.70° | 0.648 | 0.141 | 491 | 6% |
| dampingProbeInterval 2 | 10.738 | 4.70° | 0.648 | 0.141 | 463 | 11% |
| **dampingProbeInterval 3** | 10.738 | 4.70° | 0.649 | 0.141 | 441 | 16% |
| dampingProbeInterval 6 | 10.737 | 4.70° | 0.648 | 0.141 | 420 | 24% |
| dampingProbeInterval 12 | 10.737 | 4.70° | 0.648 | 0.141 | 410 | 26% |
| **ReducedFidelitySchedule** | 10.738 | 4.70° | 0.649 | 0.197 | **231** | **59%** |

## Reading it

**The flight signature is the wrong instrument for the suspension.** Trim speed
and incidence are identical to three decimals from 120 iterations all the way
down to 10 — which does not mean 10 is free, it means the signature cannot see
the difference. The network's own **residual** can: 0.140 N at 120, 0.763 N at
10. That is the number the knee is read off, and it is why the sweep reports it.

**The knee is 40.** Below it the saving flattens while the residual does not —
40% saved at 0.198 N, 49% at 0.384 N, 56% at 0.763 N. Buying the last 16% costs
nearly 4× the residual.

**Coupling iterations are not available.** Dropping 3 → 2 moves the peak of the
asymmetric collapse from 0.648 to 0.691. That is 6.5% on the number a pilot is
judged on, for 21% of a step that is already 15× faster than real time. Not a
trade worth making. 3 → 1 is worse in every column.

**The iteration cap on the frozen solves is not a lever, and this corrected the
profile.** `SOLVER_PROFILE.md` originally blamed the 600-iteration cap for the
probes' 23.8%. Dropping it to 40 saves 0–6%, inside the noise: the
warm-started probes converge and exit long before the cap is reached. What
costs is running two extra frozen solves at all — so the lever is how *often*,
not how *hard*. `dampingProbeInterval 3` buys 16% for a fold difference of
0.001, and each axis still refreshes every 0.9 s against a coefficient that
changes on the timescale of airspeed.

**Section 28's failure mode is gone, and it was checked rather than assumed.**
The probes were once wrong from being under-converged, and the symptom was not
a bad trim speed — it was a damping derivative that moved 10% between intervals
and differed between mirror-image flights. So the sweep measures roll damping
braked left against braked right at every iteration count: **-3594.2 both ways,
0.000% disagreement, at 600 and at 40**. Warm-starting fixed it, and the cap is
no longer what holds it up.

## The tier

`ReducedFidelitySchedule()` — `suspensionIterations` 40, `dampingProbeInterval`
3, everything else at the full solver's values. 231 µs/step, 36× real time.

`FullFidelitySchedule()` is the default and stays the reference: every
published number and every Level 9 calibration gate is measured on it, and a
disagreement means a disagreement with *it*.

## The caveat, stated rather than buried

The tier converges a **disturbance** more slowly, and the settled numbers hide
it. Worst network residual over a run that includes the cold start is 28 N full
against 83 N reduced — a 3× ratio, where the settled ratio is 1.4×. The flight
signature holds through it, and the gust signature is measured through a real
collapse and recovery, so this is bounded rather than unknown. But the reduced
tier is not the full solver during transients, and it should not be the tier a
published number or a new calibration is measured on.

## Not attempted

Construction, still 1021 ms, still the only measured cost a pilot would notice.
Serialising the solved polar table would remove it, and it needs a hash of the
section spec baked in and checked, or it becomes a file that can silently
disagree with the geometry that generated it.
