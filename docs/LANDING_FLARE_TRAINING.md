# Flare and run-out training

The eighth deterministic flight-lab scenario turns the landing-energy model
into a repeatable exercise. Selecting it with `Y` resets the configured wing
onto a stabilized 420 m final for the currently selected landing.

## Scenario setup

The start is 55 m above local terrain. Final heading is opposite the locally
sampled surface wind; calm conditions fall back to the selected route bearing.
The exercise therefore works at Lehn, Höhematte, Grund and Bodmi without a
hardcoded global heading.

The player must:

1. stabilize heading and sink;
2. keep sufficient hands-up airspeed and canopy pressure;
3. avoid spending flare reserve high;
4. apply a progressive symmetric flare close to the surface; and
5. retain a manageable horizontal speed for running out the landing.

HUD coaching changes from stabilization to energy preservation, setup, flare
and run-out cues using physical height and brake/flare state.

## Scoring

Touchdown produces a 1000-point result from:

- target accuracy: 200 points;
- vertical softness: 250 points;
- runnable horizontal energy: 150 points;
- first-flare height: 250 points;
- peak physical flare authority without prolonged deep brake: 100 points; and
- average approach quality: 50 points.

The timing target is centered near 2.6 m pilot AGL and fades smoothly rather
than using a pass/fail frame. This is a simulator calibration target, not a
universal real-world flare height: wing, gradient, wind, pilot geometry and
terrain all change real technique.

The landing panel reports first-flare height, peak authority and score.
Telemetry already records ground effect, flare reserve, authority and lift
increment for deeper analysis.

## Validation

Headless tests verify scenario ordering, weather, cue absence, good late-flare
scoring and a material penalty for an otherwise identical flare initiated at
20 m AGL. The scenario is included in persistent pilot progression as the
eighth mastery exercise.
