# What building this taught, and what it cost

Bugs that took real time to find, and the patterns behind them. Written so the
next person does not pay for them twice.

The through-line: **almost every serious bug in this project was a sign or a
convention, not an algorithm.** The maths was usually right the first time. What
was wrong was which way something pointed, or what a number meant, or which of
two descriptions of the same thing was authoritative.

---

## 1. The tests were compiled out

`Tests/` built with `-DCMAKE_BUILD_TYPE=Release`, which defines `NDEBUG`, which
turns every `assert()` into nothing. **605 assertions in `PhysicsTests.cpp` had
been inert.** The suite passed by doing nothing, and had done for a long time.

Turning them back on immediately surfaced: a wind-direction bug, six stale route
expectations, and two selectable routes that put the pilot off the map.

**The tell** was a compiler warning — `unused variable 'weightShift'` — for a
variable used only inside asserts. That warning had been visible the whole time.

**Rule:** a test suite that has never failed is not evidence of anything. Break
it deliberately and check it notices.

---

## 2. Conventions documented backwards, and the cost compounds

`ParagliderCoordinateSystem.h` stated all three rotation senses. All three were
wrong:

| documented | actual |
|---|---|
| roll (+X) right wing down | right wing **UP** |
| pitch (+Y) nose up | nose **DOWN** |
| yaw (+Z) nose left | nose **RIGHT** |

The flight frame is forward/right/up, which is left-handed, while the quaternion
algebra is right-handed. Nobody reconciled them; each author who found a sign
wrong fixed it *downstream* against the header instead of against the wing.

The result was a model where weight shift banked the wing right and flew it
left, and brake banked it left and flew it right — each control with one correct
half and one inverted, and the two compensating well enough that it shipped.

**Two of my own attempted fixes were also wrong**, because I reasoned from the
header. Only integrating the attitude directly and measuring settled it.

**Rule:** derive signs from a measurement, not from a comment. For anything
attitude-related, read it off the rotated basis vectors:

```cpp
incidence (nose-up +) = asin(attitude.Rotate({1,0,0}).z)
bank (right tip down +) = asin(-attitude.Rotate({0,1,0}).z)
```

---

## 3. One variable, two opposite physical effects

`lateralLoadImbalance` carried both:

- **carabiner tension** asymmetry — the loaded side is pulled **down**
- **aerodynamic lift** asymmetry — the lifted side goes **up**

One sign was applied to both. Whichever control the coefficient had been fitted
to worked; the other was inverted. No downstream sign fix could rescue both,
which is why the bug survived so long — every local fix broke the other control.

**Rule:** if two effects have opposite signs, they are two variables. Merging
them looks like simplification and is actually a permanent trap.

---

## 4. Counting the same force twice

Appeared three times, in three subsystems:

- **Level 3:** the payload roll moment was both a hang-angle target *and* an
  injected acceleration. It pinned the relative roll at its clamp, and the wing
  tipped over ten seconds into a turn.
- **Level 4:** the section's own bound vortex induced on its own control point,
  when the 2D polar *is* that circulation. It halves the lift-curve slope,
  exactly and quietly.
- **Level 4 again:** having excluded the self-panel to fix that, its trailing
  legs went too — and those must stay, because the wake carries only the
  spanwise *change* in circulation. Removing them made the cancellation
  one-sided and the induced velocity enormous.

**Rule:** when coupling a table-driven model to a field solver, ask what the
table already contains. A polar is not a boundary condition.

---

## 5. Convergence measures that measure the wrong thing

Two variants, both of which produce a solver that lies about itself.

**The residual measured the damped step**, not the equation error. Increasing
damping shrank the number, so a solve damped to a crawl reported itself
converged. Measure the residual *before* relaxation.

**A stalled iteration looks exactly like an equilibrium.** The membrane's
kinetic energy fell to zero and the shape stopped dead — at 91 mm where the
analytic answer was 26. Energy going to zero proves nothing. The tell was that
the answer moved with *iteration count* rather than with time.

**Rule:** the convergence test is "does the answer change when I change the
budget", never "has it stopped moving".

---

## 6. Solver devices that change the physics

Fictitious mass is a legitimate and very effective device: the per-substep push
is F·h²/2m, so mass reduces it exactly as substep count does, and mass is free.
A 10⁴ mass scale took the membrane from unconverged to exact.

Then it sagged the strip under 10⁴ times its own weight, because gravity was
applied to the scaled mass.

**Rule:** a numerical device may change how fast something settles. The moment
it changes *where* it settles, it is a physics change wearing a device's
clothes. Compute physical forces from physical quantities and only accelerate
against the fictitious ones — and assert the answer is unchanged when the device
is varied.

---

## 7. Stiff systems: the timestep is usually not the problem

Recurring pattern — a system that will not settle, where the instinct is more
iterations:

- **membrane:** 2.7×10⁶ N/m against 0.46 g per node puts fabric elastic waves at
  12 kHz. Nobody cares about 12 kHz. Resolving it was all the solve was doing.
- **Level 7 damping, twice.** First: roll damping has a 20 ms time constant and
  was being held across a 100 ms aerodynamic interval — damping applied at a
  rate five time constants stale, which is not damping but excitation. Diverged
  to NaN in five seconds. Evaluating it every step fixed that, and left the
  second one, which took another round of work to see: the term was still being
  integrated *explicitly*. Yaw damping of 80 Nm per rad/s on an inertia of 150
  needs `c·dt/I < 2` to be stable and sat at eleven times that, so it alternated
  sign and doubled every step. Asymmetric brake reached an infinite turn rate in
  fourteen seconds.

  The fix is one divide. Backward Euler on the damping term alone —
  `omega' = (omega + M/I·dt) / (1 + c/I·dt)` — is unconditionally stable at any
  coefficient, and leaves everything else explicit.

**Rule, sharpened:** a stiff term does not have to set the timestep. Ask which
*term* is stiff, and integrate that one implicitly. Making the rate live is
necessary and is not sufficient.

**Rule:** compare every subsystem's own time constant against the rate you
intend to run it at, *before* choosing that rate. Where they conflict, either
split the load into held and live parts, or make the fast dynamics artificially
slow, or solve it statically. Do not simply iterate harder.

---

## 8. Getting the object wrong

The membrane's first version was a chordwise loop — nodes around the section,
pinned at leading and trailing edges. It inflated to a circle regardless of the
profile it was cut to, because that is what a closed loop of fabric with
pressure inside *is*.

The fabric works spanwise. Ribs hold the profile; the skin bulges between them.

The same class of error, elsewhere: applying the aerodynamic resultant in **body
axes** at a level with no aerodynamic solver. A nose-up rotation carries the lift
round with it and the pitch equilibrium runs away instead of standing still.

**Rule:** before solving, state what physically holds the thing in place. If the
answer is "nothing", the topology is wrong and no amount of solver work fixes
it.

---

## 9. Six solvers means six initial conditions

The Level 7 coupled solve stalled and fell out of the sky, and the cause was in
none of the solvers.

The flight state started **flying** — 10.8 m/s, level, in trim. The cell
pressure state started **packed** — every cell empty, because that is the honest
default for a canopy that has not inflated yet. On the first aerodynamic
re-solve the VSM was told every cell was at zero internal pressure. Level 5's
coupling then correctly cut section lift to 15% and added a large drag penalty,
and the wing correctly stalled.

**Every subsystem did exactly the right thing.** The aggregate was a wing doing
10 m/s with no air in it, which is not a wing.

What makes this class hard is that it is invisible from inside any one solver.
Each had a defensible default. Only the combination was nonsense, and nothing
owned the combination.

**Rule:** when state lives in several subsystems, "start the simulation" is its
own operation with its own correctness condition — every piece of state
consistent with the same instant. Give it a name (`SeedInflated`), call it
explicitly, and be suspicious of any subsystem whose default is "the beginning
of its own story" when the caller is starting mid-story.

A second, cheaper instance of the same thing in the same commit: the coupled
solver was not applying installed drag, so it glided at 14 where the wing glides
at 9.5. Level 4 had computed it correctly and Level 7 simply never asked. An
integration layer can be wrong purely by omission, and omission does not throw.

---

## 10. Degenerate cases with no meaningful answer

A dead calm has no wind direction. `atan2(0, 0)` returns 0 or 180 depending on
the *sign of zero*, and the preflight briefing charged the launch a 48-point
cross-wind penalty for being cross-wind in still air.

**Rule:** where a quantity is undefined, say so explicitly and let callers check
the magnitude. Do not let a formula answer a question that has no answer.

---

## 11. Validate against something external

The checks that actually caught things were the ones with an outside reference:

- lifting-line theory for the VSM — **0.2% on CL_α**
- the circular-arc solution for the membrane — **26.32 mm vs 25.99**
- published best glide — **9.46 vs 9.5** for the wing alone, and **9.5** again
  from the fully coupled solve, which also lands trim speed at 38.5 km/h against
  a published 39
- published total line length — **254.8 m vs 254**
- rigid-body statics for carabiner loads — exact

Every one of these caught a real error. Checks written against the code's own
previous output caught none, and in several cases *encoded the bug*: the Level 1
arc had two sign errors that the tests passed because they had made the same
wrong assumption.

**Rule:** prefer a check against theory, published data or a closed form. A
golden-value test only pins behaviour; it never tells you the behaviour was
right.

---

## 12. What honest failure looks like

Three things were committed *not working*, deliberately:

- the membrane's first attempt, with the diagnosis
- deep stall in the VSM, as a locked known-failure check
- Level 7's unstable trim

In each case the alternative was a green suite asserting a wrong number — the
membrane would have pinned a 91 mm bulge where physics says 26. A test that
passes on a wrong answer is worse than no test, because it actively defends the
error.

The known-failure locks are worth the pattern: `Check(!converged, "KNOWN
FAILURE: ... delete this check when it does")`. It fails loudly when someone
fixes it, which is exactly when you want to be told.

Two of the three are now closed, and closing Level 7's told us something about
the third kind of honesty. The excluded suite carried a written suspicion — "not
reproducible by a direct probe, so the suspect is the test harness rather than
the solver, but that is unverified". It was wrong, and the word doing the damage
was *unverified*: a plausible diagnosis, recorded as a caveat, kept the bug
parked for a whole level. A direct probe at -O0 reproduced it on the first run.

**Rule:** an unverified diagnosis is not a finding, and writing it down next to
the defect makes it look like one. Either verify it or record only the symptom.

---

## 13. Measuring a derivative is its own numerical problem

Three separate defects in Level 7 were all one mistake: treating "measure how
the moment changes with rotation rate" as free.

- **Dividing by the state.** The estimator took the live moment difference over
  the live rotation rate, with a guard zeroing it below 10⁻³ rad/s. Near zero
  rate that is noise over nothing, and worse, the guard made the damping law
  *discontinuous in the state*. Two mirror-image flights took opposite branches
  of it in the fourth second and stopped being mirror images.
- **A one-sided difference.** Replacing it with a fixed ±0.3 rad/s probe fixed
  the conditioning but, probed on one side only, still measured a different
  coefficient for a left turn than for its mirrored right turn — a one-sided
  difference is not odd in the rate. Centring it restored mirror symmetry to
  2×10⁻⁸ rad after ten seconds.
- **Probing a different object.** The probes called the cold solve capped at 40
  iterations, where cold needs ninety, and got the equilibrium separation for
  whatever incidence they landed on rather than the wing's actual separation
  state. So the derivative described a wing that did not exist, and moved 10%
  between consecutive intervals. Holding the live separation state and
  continuing each probe's own circulation made it converge in a handful of
  iterations — the suite got about ten times faster as a side effect.

**Rule:** a finite difference needs a perturbation *you* choose, centred, taken
about the state you actually have, with both solves converged. If a solve is
warm-started or iteration-capped for speed, a derivative built from it inherits
that error amplified by one over the step.

---

## 14. Every accepted quantity needs a bound, not just the one that bit you

The coupled solver rejected an aerodynamic solve whose *force* exceeded fifty
times weight. It never bounded the moment. One step near stall returned a
converged solve carrying 34 kNm of yaw against a `q·S·b` of 14 kNm; it was
accepted, and then every subsequent step failed and the safety envelope
faithfully held that number for ten seconds. 26 kNm on an inertia of 150 is a
turn rate of 100 rad/s, and then infinity.

The envelope worked exactly as designed and made things worse, because "hold the
last value" is only safe if the last value was checked. And the check that
mattered was missing on the quantity nobody had seen fail yet.

**Rule:** a validity gate covers a *state*, not a symptom. List everything the
gate lets through and give each one a bound with physical units behind it — here
`q·S·b`, which was already available. And a fallback that holds a previous value
must be as suspicious of that value as of the new one.

---

## 15. A coordinate is a claim about the world

Three separate defects, one root: a position that meant something different to
two parts of the program.

- **The frame flip.** The terrain frame defined +Y as route-left while the
  flight frame defined it as route-right, and nothing converted. The whole
  surveyed landscape was mirrored about the route axis. It was invisible in
  play because the mirroring was *uniform* — pull left, bank left, watch the
  world turn left — and only the relationship to real geography was wrong,
  which is precisely what ridge lift, rotor, wind bearings and circuits are
  built on.
- **The invented lane.** Two routes were translated as a group onto `y = -8500`
  to keep them off the surveyed grid, preserving their intra-valley geometry
  while placing them 20 km from the valley. The terrain there was an analytic
  proxy that put a 950 m landing field at 4683 m.
- **The Interlaken-relative weather.** Thermal triggers and every authored
  weather volume were expressed relative to one launch, with a single lane
  offset standing in for "somewhere else". Somewhere else was dead air.

**Rule:** a coordinate system is not a convention to be recorded, it is a claim
that has to be *checked against something outside the program*. The check that
finally held this together tests both halves in one place: that left brake turns
the wing toward -Y, **and** that the lake which is really west reads its real
surface elevation on that side. Either half alone passes happily while the world
is mirrored.

**Corollary:** never special-case one route's or one site's coordinates. Project
from the anchor. Every special case here was a 20 km error wearing a comment
explaining why it was fine.

---

## 16. The datum is part of the measurement

Local z in this project is metres relative to a landing field at 565 m — not
MSL, not above ground. Two rotor samples at the same "altitude" of z = 260
compared open valley air against a point thirty metres *inside a hillside*. The
0.82-against-0.00 that produced was recorded as evidence that the airflow model
was sided against the geography, cited in three documents, and carried for 25
commits after the bug it described had been fixed.

Sampled at a common height above ground, the field is symmetric and always was.

**Rule:** before comparing two measurements, state what they are relative to. In
terrain, "same altitude" is almost never the comparison you want; "same height
above ground" usually is. And a measurement offered as evidence for a defect
should be re-taken when the defect is closed — otherwise the number outlives the
bug and keeps arguing for it.

---

## Numbers worth remembering

| quantity | value | why it matters |
|---|---|---|
| trim dynamic pressure | ~65 Pa | cell pressure, membrane load |
| cell pressure coefficient at trim | 0.97 | inlets sit near stagnation |
| line stretch at trim | 0.5 cm | small but not negligible |
| membrane strain at trim | 0.06% | the skin is effectively inextensible |
| installed drag | 47% of canopy drag | not a correction term |
| apparent mass, normal | 33.6 kg | a third of the aircraft |
| pendulum stiffness | ~7000 N·m/rad | dominates pitch and roll |
| roll damping time constant | 20 ms | sets the maximum aero interval |
| coupled trim speed | 10.70 m/s | 38.5 km/h against a published 39 |
| coupled sink and glide | 1.12 m/s, 9.5 | published 1.0 min sink, glide 9.5 |
