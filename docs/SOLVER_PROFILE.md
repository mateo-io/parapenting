# Coupled solver profile

Level 10's first strand. Solver levels of detail are the second, and a cheaper
tier is a guess until the expensive stage is known — so this measures first.

```bash
Intermediate/PhysicsTests/parapenting_solver_profile
```

It is not part of `Tools/check-build.sh` and it asserts nothing. Wall clock is
a property of the machine, not of the model, and gating on it would turn a busy
laptop into a red suite. The instrumentation lives in
`CoupledParagliderSolver` behind `SetProfiling`, off by default, costing one
predictable branch per stage when off.

## Machine

Apple M1 Max, macOS 15 (Darwin 25.3.0), Release (`-O2`), single-threaded.
Measured 2026-08-03 at commit `a22611f`.

Each case settles for 20 s **unprofiled** — the first steps of a fresh solver
run a cold 12000-iteration suspension relaxation and a 600-iteration cold VSM,
which are real costs but are not the steady flight loop — then 30 s of flight
is measured.

## Per step, hands-up cruise

| stage | µs/step | share |
|---|---|---|
| suspension | 328.0 | **60.7%** |
| VSM damping probes | 128.6 | **23.8%** |
| VSM unsteady | 33.0 | 6.1% |
| VSM stationary | 32.9 | 6.1% |
| membrane | 9.2 | 1.7% |
| collapse | 1.1 | 0.2% |
| rigid motion | 0.6 | 0.1% |
| cell pressure | 0.1 | 0.0% |
| unaccounted | 6.6 | 1.2% |
| **total** | **540.0** | |

## Four things this says

**1. It is already real time, with 15× to spare.** 540 µs against a 8333 µs
budget at 120 Hz — 6.5% of one core. The solver was never the reason the game
flies `ParagliderDynamics`; guiding rule 11 was. That reframes what levels of
detail are *for*: not making this playable, but weaker machines, more than one
aircraft, and faster-than-real-time research runs.

**2. The cost is the lines, not the air.** Everyone's assumption — including
the one implied by `aerodynamicsInterval` existing at all — was that the
aerodynamics dominate. They are 36%. The suspension network is 60%, because it
is the one expensive stage that runs **every step and every coupling
iteration**: 120 relaxation sweeps × 3 couplings = 360 sweeps per step, about
0.91 µs each. The aerodynamic side is already amortised 12:1 by the schedule
and the line network is not amortised at all.

**3. The damping probes cost four times the solve the wing flies on.** An
aerodynamic tick runs four VSM solves: the unsteady one, a rotation-free one
for the moment, and two rate probes for the damping derivative. The probes are
23.8% of every step against the unsteady solve's 6.1%.

> **Corrected by the strand 2 sweep.** This section originally went on to
> blame the 600-iteration cap, against the unsteady solve's 40. That was wrong,
> and it is the reason strand 2 measured before cutting. Dropping the cap from
> 600 to 40 saves **0–6%, inside the run-to-run noise**: the warm-started
> probes converge and exit long before the cap, so it is never reached. What
> costs is running two extra frozen solves *at all*, which makes their
> **frequency** the lever, not their iteration count. See
> `SOLVER_LOD.md`.

**4. Nothing is state-dependent.** Cruise 540, 25% brake 540, asymmetric 524,
4 m/s gust 552 — a 5% spread across flight states that load the stages very
differently. So a level-of-detail scheme does not need to detect "near the
edge" and switch tiers; a fixed cheaper tier is enough, and the complexity of
switching buys nothing.

## The one number that is a problem

**Construction: 1021 ms.** Section polar table over 21 brake stations, each a
panel factorisation and an incidence sweep, plus the suspension graph, trim
load distribution, line stiffness curve and brake swing curve — all solved
rather than loaded. That is a second of stall to swap a wing, and it is the
only measured cost here that a pilot would ever notice.

## What strand 2 did

See `SOLVER_LOD.md`. In short: `ReducedFidelitySchedule` moves two knobs and
buys **59%** of a step, 522 → 231 µs, with trim speed and incidence unchanged
to three decimals and the 4 m/s asymmetric collapse moving 0.001. It is gated
against the full solver in `coupled_tests` on every run.

Of the three targets below, target 1 delivered, target 2 delivered but not for
the reason given, and target 3 was not attempted.

## What strand 2 should act on, in order

1. **Suspension iterations and coupling.** 60.7%, and the only stage with no
   amortisation. Two independent levers — `suspensionIterations` (120) and
   `couplingIterations` (3). The convergence gate already stated in
   `CoupledSchedule` is that taking one coupling iteration away changes nothing
   qualitative; `coupled_tests` measures 10.928 m/s at 3 against 10.871 at 2.
   A tier must be checked against the full solver, not asserted.
2. **The damping probe iteration cap.** 23.8% for a derivative that is
   differenced over ±0.3 rad/s. Whether it needs 600 iterations or the
   unsteady solve's 40 is a measurement nobody has taken — and §28 of
   `PHYSICS_LEARNINGS` is the warning attached to it: these probes were
   *already* wrong once from being under-converged at 40 iterations cold, which
   is why the cap is 600 and why they are warm-started now. Warm-started, the
   answer may be different. Measure the derivative against iteration count
   before touching it.
3. **Construction.** Serialising the solved polar table would remove ~1 s of
   wing swap. It also introduces a file that can disagree with the geometry
   that generated it, which is the kind of drift the computed table was
   introduced to end — so it needs a hash of the section spec baked in and
   checked.

Nothing below the membrane's 1.7% is worth touching. Pressure, collapse and
rigid motion together are 0.3% of a step.
