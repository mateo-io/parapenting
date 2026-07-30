# Aerodynamic collapse-onset model

Canopy incidents are initiated from the same local aerodynamic state used by
the flight solver. Authored rotor strength remains one possible disturbance,
but it is no longer the sole trigger.

## Unloading signal

At every fixed 120 Hz physics step the solver measures:

- wing-relative airflow and incidence;
- dynamic pressure and its rate of loss;
- low-pressure exposure;
- turbulence and lateral-gust magnitude;
- current wing loading and internal canopy pressure.

Loss of incidence, a rapid pressure drop, or low dynamic pressure creates an
unloading signal. Incidence or control movement in smooth air is deliberately
insufficient on its own for ordinary values; this prevents active piloting and
surge checks from spuriously retriggering a collapse. Disturbed air magnifies
the same physical unloading.

## Incident direction and type

The sign of the lateral gust biases the affected half of the canopy. The
opposite half receives a smaller drive, allowing asymmetric folds rather than
binary scripted incidents. Strong symmetric unloading contributes to a
frontal. Higher wing loading supplies a modest pressure-resistance term.

## Wing-specific response

Every wing profile carries a separate research envelope for:

- collapse resistance under the same unloading event;
- passive, brake-assisted and pump-assisted asymmetric reinflation;
- frontal reinflation;
- cravat susceptibility;
- recovery-surge energy.

The training A is deliberately the most resistant and fastest reopening
profile. The sport B is the most disturbance-sensitive, slowest passively
reopening and most surge-prone. The BGD EPIC 2 retains the original baseline.
The ADVANCE EPSILON DLS research profile is tuned toward the damped
basic-intermediate behavior described by ADVANCE, but its numerical collapse
and recovery values are simulator estimates, not manufacturer measurements.

Once initiated, panel lift/drag loss, brake unloading, cravat probability,
heading change, recovery pumping, reinflation and recovery surge continue
through the existing spanwise canopy and incident-state models.

## Validation

Deterministic tests verify:

- eight seconds of smooth pressurized flight produces no spontaneous fold;
- disturbed rapid pressure loss produces a frontal and correctly sided fold
  without requiring an authored rotor volume;
- pulse-and-release recovery beats a held deep brake;
- pressure-gated surge checking reduces the peak recovery surge;
- all six profiles have bounded, distinct stability/recovery coefficients;
- training-A asymmetric and frontal incidents reopen faster than sport-B;
- a sub-saturating identical rotor pulse produces less folding with higher
  configured collapse resistance;
- coefficients and instantaneous reinflation authority are present in CSV
  telemetry for instrumented comparison;
- sixty ten-minute wing/weather simulations stay finite.

This remains a handling research approximation. Manufacturer flight-test data
and external SIV pilot review are required before any profile can represent a
certified real wing.
