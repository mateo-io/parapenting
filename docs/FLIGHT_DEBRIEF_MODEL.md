# Flight debrief and analysis

The flight debrief converts the 120 Hz simulation stream into stable,
human-readable flight metrics. It is engine-independent and uses threshold
crossings to count incidents, so a collapse held across hundreds of frames is
one event rather than hundreds.

## Segmented phases

- launch;
- glide;
- thermal;
- rotor;
- incident;
- approach;
- landed.

Safety-critical phases take priority over normal glide classification.

## Summary metrics

- elapsed time, horizontal distance, altitude gain and loss;
- thermal time and altitude gain;
- rotor and severe-rotor exposure;
- maximum climb, sink, airspeed and load;
- minimum canopy pressure and time above 3.5 g;
- asymmetric collapse, frontal collapse, cravat and stall/spin event counts;
- stabilized-final time and average approach quality;
- target distance and touchdown vertical/horizontal speed.

The result card provides ratings from 0–100 for safety, efficiency, thermal
use and landing, plus a weighted overall rating. The lowest category selects a
specific practice focus. Ratings are simulator feedback, not pilot
qualification.

## Persistence

Every touchdown writes a one-row CSV into `Saved/Debriefs`. Opt-in 10 Hz
telemetry includes the current phase and all five ratings. Saving a replay also
writes a timestamp-matched `analysis-*.csv` sidecar, allowing wing, route and
weather comparisons without changing the versioned deterministic replay
format.
