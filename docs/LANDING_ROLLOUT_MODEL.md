# Landing rollout and canopy management

Touchdown no longer freezes the simulator. A deterministic post-contact model
continues at the same 120 Hz fixed step through running, falling and settled
phases.

## Touchdown classification

The rollout begins from the actual horizontal and vertical touchdown velocity
and current canopy pressure:

- a runnable arrival enters `RUNNING OUT`;
- more than 12 m/s horizontal speed or a descent faster than 4 m/s enters
  `FALLEN`; and
- an already stationary arrival enters `SETTLED`.

These thresholds are simulator calibration boundaries, not medical or
operational safety limits.

## Running phase

Pilot velocity decays from foot-contact resistance, symmetric brake input,
canopy state and the along-track surface wind. Position continues across the
actual terrain and the pilot legs animate at rollout speed. Canopy pressure
decays gradually rather than disappearing on the touchdown frame.

Strong asymmetric brake while still running faster than 7.5 m/s can pull the
pilot into the fallen phase. This gives post-touchdown brake handling a
physical consequence and prevents the “run-out” exercise from ending at first
contact.

## Fall and settle

A fallen pilot decelerates more strongly, rolls into a visible ground pose and
loses canopy pressure quickly. The canopy lowers and pitches rearward as it
deflates. A safe run settles once speed falls below 0.65 m/s; canopy pressure
then continues to decay.

Wind/fabric audio uses rollout speed and live canopy pressure instead of stale
airborne telemetry. The landing panel shows phase and runout distance.
CSV telemetry records rollout phase, runout distance and decaying canopy
pressure.

## Training integration

The flare/run-out scenario waits for the rollout to settle before committing
its persistent best score. A roughly seven-metre controlled runout receives
full rollout credit. Excessive distance reduces that credit, and falling adds
a separate penalty.

## Validation

Headless regressions cover:

- safe deterministic runout distance;
- bit-identical repeated rollout;
- horizontal overspeed and hard vertical impact;
- one-sided brake-induced loss of balance;
- fall deceleration and canopy-pressure loss; and
- rollout-specific challenge-score adjustment.

This model does not yet simulate individual foot placement, ankle/knee injury,
surface friction materials or a fully cloth-solved canopy on the ground.
