# Weather presentation model

## Ownership

`AtmosphereModel` and `WeatherSnapshot` own weather truth. The renderer only
reads their snapshots through `AParagliderPawn`; it does not sample solver
internals, change the fixed step, or write a value back to physics or replay.

The current snapshot contract supplies source identity, deterministic seed,
wind direction/speed/gust and thermal top. `CloudFieldState` supplies coverage,
development, base, thickness, drift and shadow strength. This is sufficient for
the supported visual response:

| source value | presentation response |
| --- | --- |
| diurnal state | sun direction, intensity, colour and skylight fill |
| cloud coverage/development | cloud density and bounded valley veil |
| cloud base/thickness/drift | volumetric-cloud layer and drift |
| cloud shadow strength | directional cloud shadows |
| wind/gust | water roughness/specular and windsock direction/extension |

Water response uses dynamic material instances of `M_WaterSurface`; it has no
collision, buoyancy, wind forcing or feedback channel. Windsocks sample the
same local atmosphere as the flight presentation, preserving a shared direction
without claiming that a sock is an anemometer.

## Explicitly unavailable states

There is presently no deterministic precipitation amount/type, humidity,
surface temperature, ground moisture, snowpack, leaf state or wetness field.
Therefore the visual layer **must not** infer rain, snow, puddles, wet ground,
or snow accumulation from cloud cover or wind. Cloud cover is only a bounded
aerial-perspective input. Scenic precipitation remains disabled until the
physics/weather owner supplies a replay-stable source value.

## Determinism and QA

`-VisualQAWeatherPreset=0..4` selects an authored input preset only before a
capture run starts. It resets the flight and has no player-facing mutation
path. Weather captures compare the same route, graphics tier, time and preset;
their allowed differences are pixels only. A weather pass is rejected if it
changes a headless state hash, trajectory, control input stream or terrain
height query.
