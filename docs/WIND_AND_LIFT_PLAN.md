# Wind, thermals, and what the pilot can see

The point of this game is reading the air from the shape of the ground: where
the hill makes lift, where it makes rotor, where the day makes thermals, and
what the wing does in each. Everything below serves that one sentence.

Written in the shape of the physics ladder and the performance plan: stages,
gates, tests, one knob at a time, and a level that cannot state its instrument
does not start.

## The budget this is built against, which is unusually good news

`AIR_PROFILE.md` measured it, so this plan does not have to guess:

- an atmosphere sample costs **845 ns**, and the aircraft takes **three per
  step — 2.6 µs, 0.03%** of a 8333 µs step;
- **more weather structure is free**: sample cost is 90% terrain query, and
  adding thermals, rotors or sink volumes did not move it measurably;
- **sample points are what scale**, linearly: 45 per step — one per VSM
  section — is 38.9 µs, **0.47%**;
- the simulation side of the step is **94% unspent**.

So the constraint on this work is physical honesty, not cost. That is stated up
front because it removes the usual excuse for a cheap approximation.

## What already exists, and is not to be rebuilt

Read before planning, because half of this is built:

- **Thermals already have a life cycle** — build, mature, decay; a breathing
  core, an annular sink, a surface convergence layer, a broadening transition,
  an inversion cap, coherent meander, and drift with the model wind by parcel
  age (`THERMAL_CIRCULATION_MODEL.md`).
- **Slope circulation** — anabatic and katabatic flow tied to terrain normal,
  height above ground and a signed heating proxy (`SLOPE_CIRCULATION_MODEL.md`).
- **A diurnal cycle**, a cloud field, a turbulence spectrum, per-landing
  windsocks, and an `F5` pre-flight briefing that already states wind at
  launch/cruise/landing, thermal strength and top, cloud, turbulence, rotor
  risk and rotor-zone count.
- **Weather presets** with authored thermal/sink/rotor volumes, and imported
  live weather snapshots.
- **The wing already reads three sample points** — centre and both tips — and
  turns spanwise wind difference into roll.

## What is missing, and it is specific

Three defects found by reading the terrain functions the whole wind model rests
on. Each one names a stage.

**1. `RidgeExposure` does not know the wind direction.** It is
`clamp(|horizontal normal| × 2.4)` — the *steepness* of the ground, nothing
more. So "ridge lift" today is a function of how steep a slope is, not of
whether the wind is blowing onto it. **Turning the wind 180° cannot move the
lift to the other side of the ridge**, which is the single most important thing
a paraglider pilot knows about a hill.

**2. `LeeRotorPotential` is a local slope test with an invented length.** It
multiplies "is the wind driving into this slope" by the steepness sampled
**55 m upwind** — a fixed constant, independent of ridge height, wind speed and
stability. Real rotor scales with the obstacle: it reaches **several ridge
heights downwind**, needs a threshold wind to form at all, and gets worse with
stability. A 900 m ridge and a 15 m bump currently produce the same 55 m rotor.

**3. Thermals are five authored volumes per preset, not places.** The user-facing
requirement — *some places are reliably thermic* — is right, and the way to get
it is not to keep authoring five spheres per weather preset but to **derive
trigger points from the terrain and the sun**, which makes them fixed, learnable
and different per site for reasons a pilot can read off the ground.

## Rules this plan runs on

- **A published relation beats a chosen constant.** Where the literature gives
  a form — speed-up over a hill, separation criteria, thermal drift and tilt —
  the model uses it and the test checks against it. Where it does not, the
  number is named as a chosen one, in one place, with the sweep that chose it.
- **The indicator and the physics are the same field.** No visual may read a
  separate model. A thermal marker that is drawn where a *different* function
  says a thermal is would teach the pilot a lie, and this game is a teaching
  instrument. Gated: every indicator stage has a test that the drawn quantity
  is the sampled quantity.
- **Determinism is preserved.** `determinism_tests` hashes 30 s of flight; wind
  and thermals must not introduce a source of state that breaks it, at any
  frame rate.
- **The wing is not told what to do.** Lift, roll, collapse and surge stay
  outcomes of the air arriving at parts of the canopy — the same rule the
  aerodynamics ladder has enforced since Level 4.
- **Visual changes are gated by `VISUAL_QA.md`**, and performance by
  `Tools/frame-capture.sh` against the L3 baseline (game thread 2.48 ms).
- **New suite:** `Tests/WindFieldTests.cpp` → `parapenting_wind_tests`, added to
  `Tools/check-build.sh`, because none of the existing suites own this.

---

## S1 — the wind is a field over the terrain

Replace "constant wind plus two heuristics" with flow that goes over the ground.

**What it must produce**, in order of how much a pilot depends on it:

1. **Windward lift as a consequence of geometry.** For flow that follows the
   surface, the vertical component is `w = U · ∇h` — wind speed times the
   uphill slope in the wind's direction. That is ridge lift, derived rather
   than authored, and it inverts correctly when the wind turns because it is a
   dot product with the terrain gradient.
2. **Crest speed-up.** Fractional speed-up over a hill of height `H` and
   half-length `L` goes as `ΔS ≈ 2H/L` near the summit (Jackson–Hunt), which is
   why the crest is windier than the valley and why a pilot loses ground speed
   there.
3. **Gap and col acceleration** by continuity — a narrowing between two masses
   speeds the flow through it.
4. **A lift band with a height** — lift decays with height above the slope over
   a scale set by the ridge, not a constant.

**Gates**, in `parapenting_wind_tests`:

- **The red test first, because it fails today**: rotate the wind 180° over a
  ridge and the lift band must move to the other side. Currently it cannot.
- `w` agrees with `U · ∇h` to a stated tolerance in attached flow, on the real
  swissALTI3D terrain rather than an invented hill.
- Speed-up at a surveyed crest matches `2H/L` from the terrain's own `H` and
  `L`, within a stated band.
- Mirror symmetry: a mirrored terrain and mirrored wind give a mirrored field.
- The valley floor in still air is still still — no invented wind anywhere.
- Cost per sample stays inside a stated ceiling, measured by
  `parapenting_air_profile`.

## S2 — rotor: when it forms, how far it reaches, how bad it is

The other half of reading a hill, and the one that hurts.

**Physical content:**

- **A formation threshold.** Rotor needs flow separation, which needs enough
  wind and a steep enough lee. Below it, the lee is smooth and the pilot may
  fly there.
- **A downwind extent that scales with the obstacle** — several ridge heights,
  not 55 m. This is the single change that makes big terrain dangerous and
  small terrain merely bumpy.
- **Intensity that grows with wind speed and lee steepness**, and a rough
  stability dependence (a stable, strongly-sheared day rotors worse).
- **Position downwind of the crest for any wind direction**, including when the
  wind turns during a flight.

**Gates:**

- no rotor below the threshold wind, anywhere, in a still-air control;
- rotor extent scales with measured ridge height across at least three
  surveyed ridges of different size — the test that kills the 55 m constant;
- the rotor zone sits downwind of the crest under four wind directions;
- turbulence intensity is monotone in wind speed;
- **the collapse benchmarks in `coupled_tests` still behave**: this stage
  changes the air a wing flies into, and the existing collapse gates are the
  regression net.

## S3 — thermals that belong to places, and that the wind can kill

**Triggers from the terrain and the sun, not from an author.** A trigger point
is where the ground heats fastest and the air is most likely to release:
sun-facing slope (aspect against the diurnal sun position, which
`DiurnalCycle` already provides), slope angle, elevation, and surface type from
the existing land-cover layers — rock and meadow trigger, forest and lake do
not. The result is **fixed per site, consistent day to day, and learnable**,
which is exactly what was asked for, and it differs between Interlaken and
Grindelwald for reasons the pilot can see out of the window.

**A cycle in time**, which the existing model already has: build, mature,
decay, with a release period set by heating rate. Two consequences worth
stating: a trigger that has just released is empty for a while, and a pilot who
learns the rhythm is rewarded — that is the game.

**And then what the wind does to them**, which is the new physics here:

- **Drift** — the column moves downwind with the air (this exists, by parcel
  age).
- **Tilt** — a thermal in wind leans downwind with height, roughly
  `atan(U / w)`, so the core is *not* above the trigger, and finding it means
  flying upwind of the ground marker. This is a skill and it is currently
  missing.
- **Break-up** — above a wind or shear threshold the column shears apart into
  broken, drifting lift instead of a workable core. A strong day is not a
  better day.
- **Downwind release** — the trigger's bubble is displaced by the time it
  reaches working height.

**Gates:**

- climb rate lands in a stated realistic band and does so on the diurnal cycle;
- drift equals wind × parcel age, measured;
- tilt equals `atan(U/w)` within a stated tolerance across three wind speeds;
- above the break-up threshold no coherent core exists — measured as the
  largest sustained climb a circling wing can find, not as a parameter;
- **same site, same time of day, same thermals** across runs and frame rates;
- the trigger set is stable and non-empty on both surveyed regions.

## S4 — the wing feels the air, not a number

Today the canopy samples three points. A thermal edge, a gust front and a rotor
are all **spanwise** events, and the wing already turns spanwise difference into
roll.

- Move to **per-section sampling** where it matters — 45 samples is 0.47% of a
  step, measured, so this is affordable.
- The existing collapse, cravat and surge machinery is what should respond;
  nothing new gets to push the wing directly.

**Gates:** entering a thermal off-centre rolls the wing toward the core without
a turn moment being applied; a rotor edge produces the collapse behaviour the
Level 8 benchmarks already describe; the cost stays within the measured budget;
determinism holds.

## S5 — seeing it, which is the point of the game

**Every indicator reads the same field the wing flies through.** That is the
rule, and it is what makes the game teach rather than decorate.

**On the ground — where thermals are being born:**

- dust, chaff and debris turning at active trigger points;
- vegetation and crop movement showing the surface convergence that feeds a
  thermal — the inflow is the tell, and it exists in the model already;
- water response on the lake for wind direction and gust fronts;
- birds working a mature thermal, which is what a real pilot actually looks for;
- heat shimmer over strong triggers.

**In the air — where the lift is now:**

- **cumulus over mature thermals**, tied to the thermal's own state rather than
  drawn independently: cloud marks the top of a working column, and it decays
  when the column does;
- drifting air motes (the component exists) coloured by vertical velocity, so a
  climb is visible before the vario says so;
- wisps and condensation near cloud base.

**Rotor, made visible, because identifying it is the survival skill:**

- lee-side turbulence made visible in the medium — dust, spray, tumbling
  debris, shimmer — so it is read from the air rather than from a HUD overlay;
- a terrain-derived hazard cue that appears only when the wind is actually
  making rotor there, and moves when the wind turns.

**The wind indicator, upper left**, deliberately simple: direction as a compass
arrow relative to the nose, speed, gust, and one terrain-relative word —
windward, lee, or crossed. The last is what turns a number into a decision.

**Gates:** an indicator test that samples the drawn location and asserts the
field there matches what is drawn; `VISUAL_QA.md` route pass; frame budget held
against the L3 baseline, with the specific rule that indicators must not
re-sample the field per particle.

## S6 — the day, and the loop that teaches it

- **A warm thermic day is the default**, because that is the game: the shipped
  preset is a working thermal day rather than the still-air lab.
- **The learning loop:** look at the wind, look at the terrain, predict where
  lift and rotor will be, fly it, and find out. The pre-flight briefing already
  states the wind; the debrief already scores flights. What is missing is the
  middle: a flight where the prediction was right or wrong for visible reasons.
- Training scenarios for the three readings that matter: work a thermal, cross
  a lee side safely, and recognise a day that has become too strong.

---

## What this plan refuses

- **Lift that is not made by air.** No thermal that adds climb rate to the
  aircraft directly; every one of these is a wind field the wing flies in.
- **An indicator with its own model.** See the rule above; this is the one that
  would quietly ruin the game.
- **Chosen constants where a relation exists**, and unnamed constants anywhere.
  `2.4`, `2.5` and `55.0` in `TerrainModel` are exactly what this plan is
  replacing.
- **Breaking the collapse gates to make the air more exciting.** If a change
  makes the wing behave differently in the Level 8 benchmarks, that is a
  result to explain, not a test to update.

## How this plan is allowed to be wrong

The performance plan's first version had two levels that were wrong because
they reasoned from structure to conclusion without a measurement in between.
The equivalent risk here is reasoning from *plausible meteorology* to a model
nobody checked against the terrain it runs on. So:

- every stage states its instrument first, and S1 opens with a test that fails
  today;
- the surveyed swissALTI3D terrain is the test fixture, not an invented hill,
  because the whole claim is that this wind belongs to *these* mountains;
- when a stage's measurement contradicts this document, the document changes
  and the old text stays visible, the way the physics ladder keeps its
  retractions.

## Order

S1 → S2 → S3 → S4 → S5 → S6, and the order is not free: S2's rotor needs S1's
field to know where the flow separates, S3's drift and tilt need a wind that
varies with position, S4 needs something worth sampling per section, and S5 must
not be built until there is a field worth drawing — an indicator for a wind
model that is about to be replaced is work done twice and a lie shown once.
