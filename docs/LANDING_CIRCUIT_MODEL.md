# Wind-oriented landing circuit

The landing aid is generated from the sampled surface wind at the selected
official landing site. Final is aligned into wind, with a deterministic
downwind, base and final gate. In calm conditions the route frame is used.

Lehn uses the published right-hand circuit in the normal valley breeze and a
left-hand circuit in mountain flow. Hoehematte uses the opposite convention.
The simulator reverses the circuit when the local flow reverses.

Approach quality combines:

- lateral displacement from final;
- ground-track alignment into wind;
- vertical speed;
- time spent on a stabilized final.

Touchdown score now reserves 200 of 1000 points for approach quality. This
means landing close to the target after an unstable, crosswind or downwind
arrival does not receive the same result as a planned circuit.

The world visualization appears within 1.7 km of the landing and colors the
downwind, base and final legs blue, amber and green. An unstable final gate is
red. These are training aids and do not represent physical site markers.
