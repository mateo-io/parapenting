# Brake travel and force model

## Travel

Brake commands remain normalized from zero to one for keyboard, controller and
replay compatibility. Physical travel is reported in millimetres:

- Short: 520 mm
- Standard: 620 mm
- Long: 720 mm

Each setup has its own free-play fraction and control gain. The researched BGD
EPIC 2 ML public envelope lists 720 mm minimum brake range at maximum all-up
mass and approximately 70 mm free play; the simulator's standard setup places
effective onset near that free-play value.

## Force

Each wing class defines a maximum research brake-force scale and a nonlinear
pressure exponent. Runtime handle force depends on:

- effective travel after free play;
- dynamic pressure, approximately proportional to airspeed squared;
- wing loading;
- canopy internal pressure;
- frontal pressure loss;
- same-side collapse and cravat unloading;
- short, standard or long control setup.

The HUD reports left and right force in newtons. Controller haptics consume the
same normalized force signal, so a loaded brake feels progressively stronger
and an affected side goes light during a collapse. Telemetry CSV files export
both normalized load and force in newtons.

## Limits

The force scales are class-inspired research values. Exact handle-force curves
depend on wing, size, loading, brake routing, pulley friction, line shrinkage,
harness geometry and rigging. Manufacturer measurements or instrumented load
cells are required before treating them as model-specific data.
