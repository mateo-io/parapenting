# Deterministic turbulence spectrum

## Spatial bands

Generic turbulence and lee rotor no longer use one scalar wave for all velocity
components. The atmosphere evaluates three analytic frozen-field bands:

- large eddies for broad displacement and load change;
- medium structures for canopy-scale gradients; and
- short structures for fabric/controller texture.

Each band combines independent three-dimensional modes. The field is advected
by the configured model wind and also evolves slowly, so flying with an airmass
does not freeze every cue while a stationary point still sees passing
structure. Mechanical energy is strongest near terrain and decays with height.
Rotor injects proportionally more energy into medium and short bands than
ordinary background turbulence.

The shortest temporal mode remains comfortably below the Nyquist limit even
at the solver's largest accepted 1/60-second step. Normal gameplay still
integrates at 120 Hz.

## Coupling

The complete velocity vector enters the flight solver and left/centre/right
canopy probes. The atmosphere separately reports:

- large-band magnitude;
- short-band magnitude; and
- resultant gust magnitude.

Large-band energy contributes to accessibility-scaled camera buffet.
Short-band energy adds subtle symmetric fabric sound and controller texture.
The actual left/right sampled velocity difference continues to drive
asymmetric panel loading and collapse onset.

All three metrics are visible in expanded diagnostics or exported CSV
telemetry, making feedback tuning traceable to physical atmosphere state.

## Determinism and limits

There is no runtime random-number state. Position, simulation time, wind and
weather parameters uniquely determine every sample, preserving exact replay.
Tests require repeated values to be bit-identical, calm mode to contain no
gust spectrum, active rotor to populate all bands, and adjacent 120 Hz samples
to remain bounded. The full wing/weather matrix checks finite long-term
behavior.

This is a band-limited procedural approximation, not large-eddy simulation or
site-resolved CFD. Mode wavelengths, amplitudes and terrain decay require
comparison with instrumented Alpine flights before validation.
