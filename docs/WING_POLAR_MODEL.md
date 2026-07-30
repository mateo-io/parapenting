# Wing polar model

## Purpose

Each wing profile contains an explicit five-point effective-brake polar. The
points represent 0, 25, 50, 75 and 100 percent brake after free play and store
lift- and drag-coefficient deltas.

The 16-panel canopy solver samples the curve independently for each side. A
single-sided input therefore changes local lift, drag, roll moment, yaw moment
and spanwise loading instead of applying only a global turn torque.

## Shape

The curves model four handling regions:

1. Free play is removed before the polar is sampled.
2. Early brake increases lift with little drag and covers the useful
   minimum-sink region.
3. Mid brake trades speed and glide for stronger lift and turn authority.
4. Deep brake develops a rapidly increasing drag wall before the separate
   time-dependent deep-stall state takes over.

Wing size and short/standard/long brake setups scale the coefficient curves
along with brake moments and 520/620/720 mm physical travel.

## Validation

`EstimateSteadyPolarPoint` solves the quasi-steady force balance for any brake
position. Automated checks verify for all six profiles:

- finite and monotonic control curves;
- plausible trim speed and glide;
- decreasing equilibrium speed with brake;
- nonlinear deep-brake drag and glide loss;
- the published BGD EPIC 2 ML research targets within declared tolerances;
- the official ADVANCE EPSILON DLS 28 geometry, loading and provenance plus
  the separately labelled simulator-estimated polar;
- deterministic stability across sixty ten-minute wing/weather runs.

Dynamic flight still runs through the full 120 Hz state integrator with
pendulum, risers, accelerator compliance, rotor, collapses, cravats, surge and
the spanwise panel model.

## Limits

Except for public envelope values explicitly identified in the wing data,
these curves are class-inspired research approximations. They are not measured
manufacturer polars. Exact minimum sink, accelerated polar, brake force, pitch
response, collapse behavior and recovery require controlled flight data or
manufacturer collaboration.

Manufacturer-specific source packages live in `Data/Wings`. They distinguish
published fields from simulator estimates and are research candidates only:

- `bgd-epic-2-ml-research.json`
- `advance-epsilon-dls-28-research.json`
