# Pre-flight weather briefing

The `F5` briefing evaluates the selected route against the same deterministic
atmosphere sampled by the wing. It is recomputed from current state whenever
drawn and automatically opens after route, equipment or weather changes.

## Inputs

- official-route launch facing, wind sector and simulator wind limit;
- sampled wind and turbulence at launch, cruise height and landing;
- steady wind, model gust spread and thermal top;
- deterministic cloud base, coverage and development;
- authored and procedural rotor strength;
- advanced-landing designation and published site notes.

## Output

The briefing displays a 0–100 suitability score and low, moderate, high or
extreme simulator risk. Penalties are traceable to launch direction/strength,
rotor, turbulence, landing wind, advanced landing complexity and low cloud
margin. A priority recommendation explains the largest immediate concern.

Preset snapshots now carry their actual thermal top and use the shared
meteorological/local-frame conversion. Manual wind changes update direction,
speed and gust metadata atomically, preventing stale briefing values.

This feature is explicitly a simulator planning tool. Live model data, authored
presets and the risk score are not current official site status, an aviation
forecast, or a substitute for qualified instruction and real-world decisions.
