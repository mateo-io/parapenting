# Deterministic diurnal weather and light

Weather and rendering share one replay-safe local-time model. Each authored
preset starts at a representative hour, and one local hour passes in 600
seconds of simulation. `F11` advances the selected start time by three hours
and resets the flight so a session never changes its initial conditions
mid-replay.

## Shared state

`EvaluateDiurnalCycle` produces:

- wrapped local clock time;
- solar elevation and azimuth;
- signed surface heat flux;
- normalized convective activity;
- ambient daylight; and
- warm sunrise/sunset light.

The function depends only on the selected start hour and deterministic
simulation time. Atmosphere samples, cloud presentation, lighting, telemetry
and replay therefore observe the same phase.

## Atmospheric coupling

Daytime convective activity scales structured thermal sources and authored
thermal volumes. Cloud coverage and layer development follow the same
activity. Ground heat flux lags solar noon: anabatic circulation builds after
sunrise, remains active into late afternoon and reverses into katabatic
drainage after sunset. Ridge lift, synoptic/model wind and explicitly authored
rotor remain available independently of solar heating.

The model does not abruptly switch at a clock boundary. Smooth transitions
avoid nonphysical impulses in canopy-relative airflow.

## Visual coupling

Unreal rotates its atmospheric directional light from the same solar angles.
Intensity follows ambient daylight, the light warms near the horizon, and
cloud-shadow strength fades with daylight. The HUD and pre-flight briefing
show local time.

## Replay compatibility

Native replay format version 2 stores the selected start hour. The loader
continues to accept version 1 files, deriving their hour from the recorded
weather preset. Enum validation includes all five current presets.

## Limits

The solar path is a representative Bernese Oberland training-day curve, not an
astronomical ephemeris. Sunrise and sunset are not yet date-dependent. The
accelerated clock is designed to expose transitions during testing and does
not claim to reconstruct historical weather.
