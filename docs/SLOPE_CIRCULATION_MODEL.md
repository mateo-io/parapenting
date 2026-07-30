# Alpine slope-circulation model

The atmosphere includes deterministic terrain-following flow in addition to
model wind, thermals, ridge lift, rotor and gusts. This gives low passes and
landing approaches a local air mass tied to the shape and heating state of the
terrain.

## Physical contract

`EvaluateTerrainCirculation` receives the local terrain normal, height above
ground and a signed surface-heating proxy:

- positive heating produces daytime anabatic flow uphill;
- negative heating produces evening or nighttime katabatic drainage downhill;
- flat ground produces no slope-flow component;
- the response increases smoothly with terrain grade; and
- the circulation decays exponentially through a 145 m near-surface layer.

The horizontal component follows the terrain gradient. A smaller vertical
component follows the slope angle, so daytime flow rises and drainage descends
without becoming a substitute for coherent thermal lift. Strength is bounded
below roughly 1.6 m/s in the current presets.

The resulting velocity is added before ridge-lift and canopy-span sampling.
Consequently each half of the canopy can encounter a different terrain-flow
vector near a sharp slope transition, and every landing windsock samples its
own local circulation.

## Weather coupling

Each authored weather preset stores surface heating separately from thermal
strength. `ACTIVE THERMAL DAY` has strong anabatic flow, `VALLEY BREEZE` has a
moderate response, and strong Foehn suppresses local heating circulation. The
selectable `EVENING DRAINAGE` preset uses weak ambient wind, almost no thermal
activity and signed negative heating to expose katabatic behavior.

Imported weather derives a conservative positive heating proxy from its
thermal-top estimate. It does not attempt to infer a real diurnal heat budget.

## Validation and limits

Headless tests verify uphill/downhill reversal, vertical sign, flat-ground
suppression, altitude decay, deterministic integration and active drainage on
the Grindelwald terrain proxy. The complete wing/weather regression matrix
then checks long-run stability.

This is a compact gameplay-scale boundary-layer model. It does not solve
radiation, surface energy balance, cold-pool depth, valley-volume circulation
or transient flow separation, and it is not operational weather guidance.
