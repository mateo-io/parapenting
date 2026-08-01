# Physics: what is left

Every open physics item in one place, with what blocks it and what "done" looks
like. The specification and per-level detail live in
`agent-data/GEOMETRY_DRIVEN_PARAGLIDER_MASTER_PLAN.md`; what is built lives in
`docs/PHYSICS_ENGINE.md`; what it cost is in `docs/PHYSICS_LEARNINGS.md`.

Status as of the Grindelwald region landing. Levels 0-7 are done, all ten test
suites green, and nothing geometry-driven flies the wing yet.

## Blocked on things this environment does not have

**1. Section polars are analytic.** Thin-airfoil lift with the circular-arc
zero-lift angle, brake as a trailing-edge flap, Viterna-Corrigan post-stall.
All derived, all registered `Provisional`. Every flight number in Level 4 rests
on theory rather than measurement.

- Blocked by: no XFOIL, aerosandbox or neuralfoil available here. Checked.
- Needs: XFOIL (or equivalent) runs over digitised EPIC 2 ML profiles at the
  Reynolds numbers the sections actually see, swept over incidence and brake
  deflection, tabulated into `SectionPolarTable`.
- Done when: the registry's `Provisional` polar coefficients become `Measured`,
  and Level 4's lifting-line validation still holds with real section data.
- Can run in parallel with anything. Long lead time. Start it early.

**2. Apparent-mass rotational terms are disputed.** The leading coefficients
could not be checked against the source paper and disagree with the model's own
estimate by a factor of fourteen in roll.

- Blocked by: no access to the source paper.
- Needs: the Lissaman and Brown derivation, or an independent derivation.
- Done when: the two `Disputed` registry entries resolve to `Provisional` or
  better, with the disagreement explained rather than averaged.
- Nothing currently uses their magnitude, so this is not urgent — but it will be
  the moment rotational dynamics are calibrated.

## Level 8 — emergent collapse (next)

**3. Level 6 is one-dimensional.** Spanwise strips at chord stations, ribs as
fixed endpoints, **no self-collision**.

- This is the gate on Level 8: a cravat is fabric touching fabric, and a
  membrane that cannot collide with itself cannot produce one.
- Needs: self-collision in `CanopyMembraneSolver`, and probably a move from
  strips to a 2-D mesh.
- Done when: a cravat forms and clears from membrane mechanics, with no
  scripted folding anywhere (guiding rule 6).

**4. Collapse must stay emergent.** The legacy model has stateful scripted
collapses. Level 8's version must arise from local unloading, pressure loss and
membrane deformation, and the two must run side by side until the new one is
better (guiding rule 11).

## Deliberate limitations, not bugs

**5. Deep stall does not converge in the VSM solved cold**, and will not. The
separated branch has a negative lift slope, which inverts the downwash feedback
between sections; a wing in deep stall has no stable steady state to find.

- Locked as a known-failure check: `Check(!converged, "KNOWN FAILURE: ...")`,
  which fails loudly if someone ever fixes it.
- The honest treatment is Level 11's unsteady wake.
- Note this is about the *cold* solve. Inside the coupled solver, with Level 4's
  separation state carried between steps, the wing walks into a fully separated
  46-degree stall at 4.65 m/s of sink without the solve failing at all.

## Integration debt

**6. Nothing geometry-driven flies the wing.** `ParagliderDynamics` — one
six-degree-of-freedom body with a fitted polar — is still what the game flies.
Levels 1-7 are exercised only by their own suites and the debug views.

- This is guiding rule 11 working as intended, not an accident.
- But it means no part of the geometry-driven stack has been felt by a pilot,
  and handling feedback cannot reach it.
- Level 10 removes the legacy path. It must not start before Level 9 calibration,
  and Level 9 must not start before real section polars (item 1).

**7. Coefficient registry: 89 coefficients, 25 tuned, 78 unvalidated.** 28%
tuned, down from 39% — the geometry-driven levels replaced fitted numbers with
measured or derived ones. The remaining tuned coefficients are concentrated in
the legacy model, and item 6 is what retires them.

## Data gaps

**8. Grindelwald First's anchor is 50 m above its surveyed ground.** Published
2123 m is the top station; its WGS84 pair is on the launch slope below, which
swissALTI3D puts at 2073 m. Every other site agrees within 12 m.

- Needs: a better coordinate for the actual launch, from a source.
- Recorded with a named tolerance in `terrain_survey_tests` rather than fitted
  away — the terrain is the measurement, the anchor is the estimate.

## UNCOMPILED ENGINE CHANGES — read this first

**Deliberate decision: we have stopped attempting Unreal module builds for now.**
External Unreal build/cook/package is quota-blocked until roughly 2026-08-05, and
retrying it burns time without producing a result. Engine-side work continues and
is committed unverified, on purpose. This is the debt that buys that.

**The CMake suites do not compile a single line of engine code.** They build each
`Physics/*.cpp` as its own translation unit — exactly the configuration in which a
unity-build name collision is invisible, which has let the module stay broken for
hours while the tests stayed green. **All ten suites passing says nothing about
any file below.**

Unverified engine changes, newest first:

| commit | files | what could break |
|---|---|---|
| `fa26882` | `ParagliderPawn.{h,cpp}` | New `PilotRigToActor` / `CarabinerLocalCm` / `RiserTopLocalCm` / `BrakeHandLocalCm` helpers; `LastPilotPose` member needing `PilotPose.h` in the header; `HarnessGeometryFor(Equipment)` call site |
| `74d1b9b` | `ParapentingTerrain.{h,cpp}`, `ParapentingGameMode.cpp`, `ParagliderPawn.cpp` | `TerrainRenderLayout` went from static constants to an instance — every `Layout::` use had to become `Active.`; new `EngineUtils.h` and `ParapentingTerrain.h` includes in the pawn; `TActorIterator` in `ResetFlight` |

**When the quota lifts, before anything else:**

```sh
Tools/check-build.sh          # module AND tests, in that order
```

Then a runtime smoke test, because these changes are visual and structural:

1. fly the Amisbühl → Lehn route — risers should read as four separate webbing
   bands per side, mains should fan from the riser tops, and the brake lines
   should end in the pilot's fists through the whole brake range;
2. weight-shift hard both ways — the pilot, the carabiners and the lines must
   move as one object;
3. `[` / `]` to a Grindelwald route — the terrain mesh must rebuild for the new
   region rather than leaving the pilot over empty space, and back again.

Nothing else in this file should start before that, because every later engine
change stacks on top of an unverified one.

## Closed recently, for orientation

- **Level 7 coupled solver.** Trim on the published wing, turns emerge and
  mirror to 2e-8 rad, suite green and running with the other nine. Four defects
  fixed: explicit damping integration at 11x its stability limit, a damping
  derivative divided by the live rate, probes solving a cold unconverged wing,
  and a validity gate that bounded force but not moment.
- **Terrain/flight frame disagreement.** Fixed 25 commits before anyone noticed;
  the evidence for it being open was a rotor sample taken thirty metres
  underground.
- **Both Grindelwald routes off the map.** Own swissALTI3D region at the sites'
  true projected positions. All ten routes on surveyed ground.
