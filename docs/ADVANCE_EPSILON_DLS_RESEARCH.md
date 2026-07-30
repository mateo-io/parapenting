# ADVANCE EPSILON DLS 28 research calibration

## Why size 28

The simulator defaults to an 85 kg pilot in a seated harness. With the
provisional 14 kg harness/equipment allowance and the published 4.35 kg wing
mass, simulated all-up mass is 103.35 kg. That lies inside both the published
91–118 kg certified range and 99–113 kg ideal range for size 28.

## Published inputs

The official ADVANCE product page supplies:

- EN/LTF B basic-intermediate positioning;
- 27.6 m² flat and 23.3 m² projected area;
- 11.92 m flat and 9.23 m projected span;
- 5.14 flat and 3.67 projected aspect ratio;
- 2.91 m maximum chord, 47 cells and 4.35 kg wing mass;
- 91–118 kg certified and 99–113 kg ideal takeoff weight.

Source, accessed 2026-07-30:
<https://www.advance.swiss/en/products/paragliders/epsilon-dls>

## Simulator estimates

The steady polar, trim/top speed targets, brake free play and pressure,
damping, moments, accelerated response, collapse thresholds and recovery are
research estimates. They are tuned to a plausible damped basic-intermediate
envelope and are not claimed to reproduce the real wing.

The machine-readable separation is in
`Data/Wings/advance-epsilon-dls-28-research.json`. Automated tests enforce
source provenance, published geometry and loading, default pilot fit,
monotonic brake behavior and deterministic ten-minute weather regressions.

## Release gate

Do not represent this profile as manufacturer-approved. Exact handling needs
ADVANCE cooperation plus instrumented polar, brake-load, line-load,
accelerated-flight, collapse and recovery measurements. Name, visual design
and commercial use also require permission.
