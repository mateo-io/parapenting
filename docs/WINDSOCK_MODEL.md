# Windsock model

Every selectable landing has a windsock sampling the same local atmosphere
used by the wing, six metres above terrain. Lehn, Höhematte, Grund and Bodmi
therefore respond independently to manual weather, presets, imported model
wind, boundary-layer reduction and deterministic gust bands rather than only
the HUD's model-wind value. Sock positions are simulator presentation offsets,
not surveyed real-world windsock coordinates.

The engine-independent pose contract converts that sample into:

- a downwind three-dimensional direction;
- inflation;
- length; and
- radius.

Below useful wind speed the sock shortens and droops. It progressively lifts
and inflates by 4.5 m/s. Gust energy adds two small deterministic angular
flutter modes without changing the mean indicated direction. Unreal aligns the
cone axis to the resulting vector and keeps its broad end anchored at the pole.

Tests cover calm droop, orthogonal wind directions, monotonic inflation and
bit-identical repeated poses. This is a visual landing aid in the simulator,
not a substitute for observing a real site or assessing operational weather.
