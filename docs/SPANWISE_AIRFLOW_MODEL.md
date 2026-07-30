# Spanwise airflow model

## What is implemented

The atmosphere is sampled at the canopy centre and at the physical left and
right half-span points every fixed 120 Hz physics step. Half-span is estimated
from configured flat area and the current research aspect-ratio assumption.
The sample points rotate with the wing, so a banked or yawed canopy meets the
airmass in the correct world-space orientation.

The model retains the centre velocity for whole-wing translational forces and
reports each side's velocity delta from that centre. The difference between
the outer samples is exposed as spanwise shear in metres per second. The
strongest sampled rotor and turbulence value is promoted to the flexible wing
for that step.

Each side's local velocity delta contributes independently to collapse onset.
Vertical discontinuity is weighted more strongly because a sudden loss or gain
of local angle of attack is especially relevant to leading-edge unloading.
The older signed centre-gust path remains for authored training incidents, but
a zero centre gust is explicitly symmetric.

The normalized left/right disturbance is also carried into the controller
model as a subtle same-side high-frequency texture. This cue begins at the
airflow boundary, before visible collapse onset, while remaining deliberately
weaker than the collapse impulse and sustained unloaded-brake cue.

## Why three samples first

The canopy solver has sixteen aerodynamic panels. Sampling the full procedural
atmosphere sixteen times per 120 Hz step is feasible, but a deterministic
three-point atmosphere contract separates two concerns:

1. the atmosphere must produce coherent spatial gradients;
2. the canopy must respond on the side that actually enters that gradient.

The centre-to-tip velocity gradient is interpolated across all sixteen panels.
Each strip therefore receives its own streamwise, cross-span and vertical flow,
which changes its local incidence, dynamic pressure, lift, drag, roll moment
and yaw moment. A later validation pass can selectively add more atmospheric
probes near moving collapse boundaries when their benefit is measurable.

## Deterministic validation

Headless tests require:

- repeated samples at the same pose and time to be bit-identical;
- calm-air outer-tip shear to remain negligible;
- a known thermal-edge search to find measurable spanwise shear;
- mirrored left/right flow discontinuities to produce exactly mirrored
  panel moments and collapse onset;
- reported telemetry shear to equal the atmosphere input; and
- all wing/weather combinations to remain finite and repeatable in the
  ten-minute regression matrix.

## Limits

This is a real spatial-flow coupling, not computational fluid dynamics. The
procedural atmosphere does not resolve canopy-scale vortices, line wake, or
fluid-structure interaction. Collapse thresholds and spatial weighting are
research assumptions pending comparison with instrumented flight data,
manufacturer data and expert SIV-pilot review. It must not be treated as a
certified real-wing prediction.
