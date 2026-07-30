# Cloud field model

The Unreal cloud layer is coupled to the same deterministic source cycles used
by thermal lift. This removes the early fixed 2.2 km / 7 km-deep scenic layer,
which looked similar in every weather preset and was far too deep for the
modeled fair-weather cumulus.

## State contract

`AtmosphereModel::SampleCloudField` returns:

- cloud-base altitude;
- layer thickness;
- coverage;
- strongest source development;
- surface-shadow strength; and
- wind-advected material offset.

The three thermal sources use the exact build/mature/decay lifecycle function
used by `ThermalCell`. Coverage responds to mean source lifecycle, thermal
strength and turbulence. Depth follows the most developed source. Drift uses
the same surveyed-frame wind and wraps after thirty minutes to keep material
parameters bounded without losing deterministic replay.

Morning-calm weather produces a thin residual layer with negligible shadows.
An active thermal day grows a roughly 0.75–0.85 km modeled cumulus layer.
Coverage and shadow strength evolve rather than jumping between presets.

## Unreal integration

The game mode updates at 5 Hz, independently of the 120 Hz flight integrator.
It applies the state to volumetric-cloud base/depth, dynamic material
coverage/density/wind parameters and directional-light cloud shadows.
Presentation never feeds back into atmosphere or wing physics.

## Validation and limits

Tests require calm coverage/shadow limits, active-day lifecycle variation,
bounded layer depth and bit-identical repeated samples. The complete
wing/weather regression matrix ensures the refactor does not alter
deterministic flight stability.

The cloud field is still a global procedural layer, not moisture-resolving
weather simulation. Source-specific cloud objects, overdevelopment, rain,
orographic cap clouds and licensed regional weather validation remain future
work.
