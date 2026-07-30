# Physics-driven camera feedback

The camera is a sensory instrument, not a second physics body. Flight state is
sampled from the fixed 120 Hz solver and converted into bounded presentation
offsets by an engine-independent deterministic model.

## Inputs

- filtered body-axis acceleration;
- airspeed and load factor;
- harness roll and pitch;
- turbulence, large-eddy gust energy and multi-frequency rotor texture;
- aerodynamic unloading;
- left/right collapse and cravat asymmetry;
- frontal collapse, deep stall and recovery surge.

## Response

Acceleration creates short inertial translation. Load factor compresses the
view downward, while airspeed expands field of view. Rotor combines three
incommensurate frequencies so it does not feel like a single repeating shake.
Asymmetric incidents displace, roll and yaw the camera toward the unloading
event. Frontals and recovery surges create opposing bounded pitch/position
cues. Deep stall adds a slow low-amplitude breathing motion.

The cinematic chase, close chase and pilot views retain separate base
positions and fields of view; they consume the same response model.

## Comfort

Full Motion uses the complete response. Comfort reduces inertial and rotor
motion independently. Minimal Motion retains only a very small orientation
signal and removes rotor buffet. These profiles affect presentation only and
never alter aerodynamics, controls or replay state.

Headless tests verify deterministic output, correct asymmetric direction,
surge/FOV response and that Minimal Motion remains below twelve percent of the
full incident displacement and roll in the reference case.
