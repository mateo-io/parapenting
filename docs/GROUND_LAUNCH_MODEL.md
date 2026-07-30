# Ground launch model

`GroundLaunchModel` extends the simulated flight from a laid-out canopy to
airborne dynamics without changing the validated airborne solver.

## State sequence

1. `WING LAID OUT`
2. `INFLATING`
3. `CHECK WING`
4. `A/D: TURN UNDER WING` (reverse launch)
5. `COMMIT - KEEP RUNNING`
6. `AIRBORNE` or `LAUNCH ABORTED`

The 120 Hz model derives apparent wind from surface wind and pilot ground
speed. Terrain slope assists pilot acceleration. Inflation produces progressive
canopy pressure and line load; lift increasingly unloads the runner before
takeoff. Crosswind moves the canopy off heading, while weight shift and
differential brake provide correction.

A heavily braked wing, loss beyond the controllable heading envelope, or
releasing the launch command before the committed run causes an abort. Lift-off
requires sufficient inflation, a centered wing, time stable overhead, apparent
wind, line-supported weight and released brakes.

## Player controls

- `N` places the wing on the selected launch terrain.
- `Shift+N` or controller D-pad up prepares a reverse launch.
- Hold `Space` or the controller right face button to inflate and run.
- Existing left/right brake and weight-shift inputs remain active.
- `R` retains the instant-airborne reset used by repeatable flight exercises.

During a reverse launch the pilot faces the wing while the canopy remains in
the route/wind frame. Brake commands therefore remain canopy-left and
canopy-right instead of silently swapping when the pilot turns. Once the wing
is centered overhead, return A/D to neutral and then choose a turn direction
with one A or D step. The pilot turns underneath over a finite interval and
cannot enter the committed run until facing forward.

The visual canopy moves from the ground behind the pilot to overhead as
inflation rises. Launch apparent wind, pressure and line load also feed the
audio layer. Replay control capture begins after lift-off so older airborne
replays retain their deterministic contract.

This is a simulator mechanics model, not launch instruction. Site suitability
and real launch decisions must come from current official information and
qualified instruction.
