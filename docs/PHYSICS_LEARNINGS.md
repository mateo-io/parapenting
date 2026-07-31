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
- **Level 7 damping:** roll damping has a 20 ms time constant and was being held
  across a 100 ms aerodynamic interval — damping applied at a rate five time
  constants stale, which is not damping but excitation. Diverged to NaN in five
  seconds.

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

## 9. Degenerate cases with no meaningful answer

A dead calm has no wind direction. `atan2(0, 0)` returns 0 or 180 depending on
the *sign of zero*, and the preflight briefing charged the launch a 48-point
cross-wind penalty for being cross-wind in still air.

**Rule:** where a quantity is undefined, say so explicitly and let callers check
the magnitude. Do not let a formula answer a question that has no answer.

---

## 10. Validate against something external

The checks that actually caught things were the ones with an outside reference:

- lifting-line theory for the VSM — **0.2% on CL_α**
- the circular-arc solution for the membrane — **26.32 mm vs 25.99**
- published best glide — **9.46 vs 9.5**
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

## 11. What honest failure looks like

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
