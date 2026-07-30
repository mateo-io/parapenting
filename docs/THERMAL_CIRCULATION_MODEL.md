# Thermal circulation model

Parapenting models thermals as deterministic, finite-lived three-dimensional
air masses. They are training and simulation fields, not forecasts.

## Structure

Each ground source follows a repeatable build, mature and decay cycle. The
plume expands with altitude, drifts with the model wind according to parcel
age, and meanders coherently rather than remaining a vertical cylinder.

The vertical field contains:

- a breathing buoyant core;
- an annular compensating sink region;
- a surface-fed convergence layer;
- a broadening transition below the configured thermal top;
- an inversion cap that removes lift above cloud base.

The horizontal field contains low-level radial convergence, weak outflow near
the cap, and alternating edge circulation. The edge component is intentionally
smaller than the radial flow: it supplies bank and yaw texture without making
every thermal a permanent vertical-axis vortex.

## Weather coupling

Authored presets provide their own thermal-top altitude. Imported weather
snapshots use the supplied `thermalTopMslM`, clamped to a conservative
1200–4200 m MSL range. Wind drift uses the same surveyed world-frame wind as
the aircraft solver.

The atmosphere sample separately reports dominant core strength, lifecycle
and cloud-base clearance. These values allow flight coaching, sound and
analytics to distinguish a coherent plume from ridge lift or generic gusts.
Terrain-following anabatic and katabatic air is modeled separately; see
`SLOPE_CIRCULATION_MODEL.md`. It is intentionally not counted as coherent
thermal-core lift.
The global cloud presentation uses the same source lifecycle function for
coverage, depth, wind drift and shadow development.

## Determinism and limits

Source cycles and meander are analytic functions of position and simulation
time. Replays therefore sample identical air at the same time and place.
Regression tests verify finite lifecycle, opposing low-level convergence,
edge circulation, cloud-base cutoff and deterministic repeated sampling.

The model does not yet solve atmospheric fluid dynamics, moisture transport,
latent heat or interactions between neighboring convective cells. Its purpose
is repeatable active-piloting behavior at game scale.
