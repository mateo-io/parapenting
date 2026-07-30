# Wind-corrected flight navigation

The navigation computer uses the configured wing's steady brake polar, current
all-up configuration, measured airspeed, sampled three-dimensional air mass,
and active waypoint geometry.

It solves:

- distance and route-frame bearing;
- along-track wind and crosswind;
- required crab angle and whether the crosswind can be cancelled;
- predicted ground speed and time to waypoint;
- net sink after vertical air movement;
- predicted arrival height;
- required and available ground glide ratios;
- minimum-sink, best-glide, accelerate or hold guidance.

A target is reachable only when the crosswind solution is feasible, forward
ground speed remains positive, and predicted arrival height exceeds the
waypoint margin.

Each selected route creates three virtual simulator gates: launch exit, valley
transit and landing. Horizontal and altitude capture limits prevent a high
overflight from completing the landing gate. These gates are training
constructs, not official aviation waypoints.

The active gate appears as a cyan world marker. Compact, minimal and expanded
HUDs display arrival margin; the expanded computer additionally shows bearing,
crab, ground speed and required/available glide.
